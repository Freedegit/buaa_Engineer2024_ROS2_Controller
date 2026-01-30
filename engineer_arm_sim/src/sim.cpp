#include "engineer_arm_sim/sim.hpp"
#include "engineer_arm_sim/ROS2_controller_thread.hpp"
//include multi-threading library
#include <thread>
#include"engineer_arm_sim/InverseKinematics.h"
#include"engineer_arm_sim/ForwardKinematics.h"
#include"engineer_arm_sim/ROS2_controller_thread.hpp"
// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context
// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;
// 期望关节角
// double q_des[6] = {0};
vector<string> joint_names = {
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6"
};
bool inited=false;
// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        arm_go_home(m,d);
    }
}


// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

int main(int argc, char **argv)
{
    puts("MuJoCo simulation started");
    // Initialize MuJoCo
    // mj_activate("your_license_key_path");
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    auto sim_node = rclcpp::Node::make_shared(
        "sim_node",
        options
    );
    std::string type = "kinematic";   // 兜底默认值
    if (sim_node->has_parameter("controller_type")) {
        type = sim_node->get_parameter("controller_type").as_string();
    }

// 转成 enum
    if (type == "pid")
        g_controller_type = ControllerType::PID;
    else if (type == "mit")
        g_controller_type = ControllerType::MIT;
    else
        g_controller_type = ControllerType::KINEMATIC;
    RCLCPP_INFO(
        sim_node->get_logger(),
        "Controller type = %s", type.c_str()
    );

    std::string xml_path = ament_index_cpp::get_package_share_directory("engineer_arm_sim") + "/mjcf/engineer_arm.xml";
    const char* xml_path_cstr = xml_path.c_str();
    m = mj_loadXML(xml_path_cstr, NULL, NULL, NULL);
    d = mj_makeData(m);
    // mj_resetDataKeyframe(m, d, 0);
    mj_forward(m, d);

    // // Set simulation time step
    // model->opt.timestep = 0.001;
    m->opt.gravity[0] = 0;
    m->opt.gravity[1] = 0;
    m->opt.gravity[2] = 0;

    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    cam.type = mjCAMERA_FREE;
    cam.distance = 5;

    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);
    // 打开关节显示
    // opt.flags[mjVIS_JOINT] = 1;
    // opt.flags[mjVIS_ACTUATOR] = 1;   // 可选：看执行器  
    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    //create a ROS2 controller thread
    std::thread ROS2_controller_thread(
        static_cast<void(*)(rclcpp::Node::SharedPtr)>(ROS2_controller_thread_func),
        sim_node 
    );
    ROS2_controller_thread.detach();


    while ( !glfwWindowShouldClose(window) ) {
        mjtNum simstart = d->time;
        
        //键盘WASD控制末端执行器
        endeffector_control_keyboard(window);
        endeffector_controller(m, d);

        //键盘数字键1-6控制各个关节
        joint_control_keyboard(window);
        joint_controller(m,d);
        
        VectorXd pos_now=get_body_pos(m,d,"link6");
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 30 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        while( d->time - simstart < 1.0/30.0 ){
    
            mj_step(m, d);
        }
        // get framebuffer viewport

        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    // Do something with simulation data

    //free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // Clean up
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_resetData(m,d);

    rclcpp::shutdown();

    return 0;
}
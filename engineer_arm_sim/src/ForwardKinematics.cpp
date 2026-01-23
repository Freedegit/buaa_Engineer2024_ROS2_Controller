#define _CRT_SECURE_NO_WARNINGS
#include<stdbool.h>
#include <iostream> 
#include "mujoco/mujoco.h"
#include "GLFW/glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <thread>
#include <chrono>
#include <Eigen/Dense>
#include <vector>
#include "engineer_arm_sim/ForwardKinematics.h"
#include "engineer_arm_sim/InverseKinematics.h"
//#include "ScoreManager.h"
using namespace Eigen;
using namespace std;

/************************************************************************************
* 定义全局变量。
* 这些全局变量在头文件中 InverseKinematics.h 中已经声明。
* 这些全局变量可以在项目中任何地方调用。
* 但同时需要注意它可能在任何地方被更改。
* 请按照需要增加或者删除。
************************************************************************************/
bool key_1 = false;
bool key_2 = false;
bool key_3 = false;
bool key_4 = false;
bool key_5 = false;
bool key_6 = false;
bool home = false;
bool space = false;
void init_model()
{
	/************************************************************************************
	* 1. init_model 函数已经在 main.cpp 中调用，在这里只需要实现即可！
	************************************************************************************/

	// 这里建议写绝对路径，否则有可能遇到Mujoco文件路径解析Bug！
	const char* xmlpath = "D:\\mujoco_env\\zhuawawa\\panda_env\\table_arm_box_scene.xml";
	char error[1000] = "Could not load binary model";
	m = mj_loadXML(xmlpath, 0, error, 1000);
	if (!m) mju_error_s("Load model error: %s", error);

	d = mj_makeData(m);
	arm_go_home(m, d);
	mj_step(m, d);
	mj_forward(m, d);
	//InitializeScoreTracker(m);
}

void arm_go_home(const mjModel* m, mjData* d)
{
	// 回家函数：让机械臂回到最开始的位置。这个位置是预先定义好的，可以任意修改。
	d->ctrl[0] = 0.0;
	d->ctrl[1] = -0.785;
	d->ctrl[2] = 0.0;
	d->ctrl[3] = -2.356;
	d->ctrl[4] = 0.0;
	d->ctrl[5] = 2.356;
	d->ctrl[6] = 0.785;
}

void joint_control_keyboard(GLFWwindow* window)
{
	/************************************************************************************
	* 1. 本函数将会在main.cpp中以60Hz的频率循环被调用，只需要实现逻辑功能即可。
	* 2. glfwGetKey函数检测是否按住了键盘中的某个按键。
	* 3. 相关按键映射可以参考官方网站：https://www.glfw.org/docs/latest/group__keys.html
	************************************************************************************/
	key_1 = glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS;  // 数字键1
	key_2 = glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS;  // 数字键2
	key_3 = glfwGetKey(window, GLFW_KEY_3) == GLFW_PRESS;  // 数字键3
	key_4 = glfwGetKey(window, GLFW_KEY_4) == GLFW_PRESS;  // 数字键4
	key_5 = glfwGetKey(window, GLFW_KEY_5) == GLFW_PRESS;  // 数字键5
	key_6 = glfwGetKey(window, GLFW_KEY_6) == GLFW_PRESS;  // 数字键6
	home = glfwGetKey(window, GLFW_KEY_H) == GLFW_PRESS;  // 字母h
	space = glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS;  // 空格键检测
	control = glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS; // 同时检测左右Ctrl键

	if (glfwGetKey(window, GLFW_KEY_BACKSPACE) == GLFW_PRESS) {
		// 如果按下backspace，仿真环境重来
		mj_resetData(m, d);
		mj_forward(m, d);
		arm_go_home(m, d);
	}
}

void joint_controller(const mjModel* m, mjData* d)
{
	/************************************************************************************
	* 1. mycontroller 函数已经在main.cpp中以60Hz的频率循环调用，只需要实现逻辑功能即可。
	*
	* 2. 在这里，此函数实现了机械臂运动的正运动学控制。即：
	*		按住键盘数字1-7，机械臂第1-7号关节转动k弧度；
	*		按住键盘CTRL+数字1-7，机械臂1-7号关节转动-k弧度。
	*
	* 3. 在这里，此函数同时实现了机械臂夹爪的控制。即：
	*		按住键盘空格键，机械臂夹爪闭合；
	*		松开键盘空格键，机械臂夹爪松开。
	*
	* 4. 允许机械臂回到最初的位置。
	*		按下键盘h，机械臂回到默认位置。
	************************************************************************************/
    double k =0.05;
	double delta;
	
	if (control)
		delta = -k;
	else
		delta = k;
	if (key_1)
	{
		d->ctrl[0] += delta;
	}
	if (key_2)
	{
		d->ctrl[1] += delta;
	}
	if (key_3)
	{
		d->ctrl[2] += delta;
	}
	if (key_4)
	{
		d->ctrl[3] += delta;
	}
	if (key_5)
	{
		d->ctrl[4] += delta;
	}
	if (key_6)
	{
		d->ctrl[5] += delta;
	}
    if (home)
		arm_go_home(m, d);
	// VectorXd current_pos = get_body_pos(m, d, "panda_link7");
	// printf("End Effector Current Positon:\n");
	// printf("x:%f\ty:%f\tz:%f\n", current_pos[0], current_pos[1], current_pos[2]);
	// printf("r:%f\tp:%f\ty:%f\n", current_pos[3], current_pos[4], current_pos[5]);
	// printf("*****************************************************\n");
}

VectorXd get_body_pos(const mjModel* m, mjData* d,const char* body_name)
{
	/************************************************************************************
	* 1. 根据 body_name 获取 body_id
	* 2. 根据 body_id 计算（并输出）物体在世界坐标系下的位姿
	************************************************************************************/
	int body_id = mj_name2id(m, mjOBJ_BODY, body_name);
	mjtNum* current_pos = d->xpos + 3 * body_id;
	mjtNum* current_rot = d->xmat + 9 * body_id;
	double roll, pitch, yaw;
	rotation_matrix_to_euler_angles(current_rot, roll, pitch, yaw);
	VectorXd body_pos(6);
	body_pos << current_pos[0], current_pos[1], current_pos[2], roll, pitch, yaw;
	return body_pos;
}

void rotation_matrix_to_euler_angles(const mjtNum* R, double& roll, double& pitch, double& yaw)
{
	/* 将旋转矩阵转换为欧拉角，roll, pitch, yaw分别表示绕x, y, z轴的旋转角度 */
	if (R[2 * 3 + 0] < 1) {
		if (R[2 * 3 + 0] > -1) {
			pitch = asin(-R[2 * 3 + 0]);4;
			roll = atan2(R[2 * 3 + 1], R[2 * 3 + 2]);
			yaw = atan2(R[1 * 3 + 0], R[0 * 3 + 0]);
		}
		else {
			pitch = M_PI / 2;
			roll = -atan2(-R[1 * 3 + 2], R[1 * 3 + 1]);
			yaw = 0;
		}
	}
	else {
		pitch = -M_PI / 2;
		roll = atan2(-R[1 * 3 + 2], R[1 * 3 + 1]);
		yaw = 0;
	}
}

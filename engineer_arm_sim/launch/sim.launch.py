# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    """launch内容描述函数，由ros2 launch 扫描调用"""
    # === 1. 控制器类型（给 sim 用） ===
    controller_type_arg = DeclareLaunchArgument(
        "controller_type",
        default_value="kinematic",
        description="Type of controller to use in sim"
    )
    # === 2. 是否启动独立 controller 节点 ===
    enable_controller_arg = DeclareLaunchArgument(
        "enable_controller",
        default_value="false",
        description="Whether to launch engineer_arm_controller"
    )
    controller_type = LaunchConfiguration("controller_type")
    enable_controller = LaunchConfiguration("enable_controller")
    # === sim：核心仿真节点 ===
    sim = Node(
        package="engineer_arm_sim",
        executable="sim",
        name="sim",
        emulate_tty=True,
        parameters=[{
            "controller_type": controller_type
        }]
    )
    # === controller（默认不启动） ===
    controller = Node(
        package="engineer_arm_controller",
        executable="engineer_arm_controller",
        condition=IfCondition(enable_controller)
    )
    # === 通信节点 ===
    communication = Node(
        package="customctrl_serial_communication",
        executable="customctrl_serial_communication",
    )
    enable_communication_arg = DeclareLaunchArgument(
    "enable_communication",
    default_value="false",
    description="Enable serial communication node"
)
    enable_communication = LaunchConfiguration("enable_communication")

    communication = Node(
        package="customctrl_serial_communication",
        executable="customctrl_serial_communication",
        condition=IfCondition(enable_communication),
        output="screen"
    )


    return LaunchDescription([
        controller_type_arg,
        enable_controller_arg,
        enable_communication_arg,
        sim,
        controller,
        communication
    ])

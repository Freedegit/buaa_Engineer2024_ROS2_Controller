clear
source install/local_setup.bash
colcon build 
ros2 launch engineer_arm_sim sim.launch.py

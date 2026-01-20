#pragma once
#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <stdbool.h>
#include <Eigen/Dense>
#include <vector>

// 声明命名空间
using namespace Eigen;
using namespace std;

// 外部变量声明
extern mjModel* m;                  // MuJoCo model
extern mjData* d;                   // MuJoCo data

// 键盘按键状态变量声明
extern bool key_1;
extern bool key_2;
extern bool key_3;
extern bool key_4;
extern bool key_5;
extern bool key_6;
extern bool home;
extern bool control;
extern bool space;

// 函数声明
void init_model();
void joint_control_keyboard(GLFWwindow* window);
void joint_controller(const mjModel* m, mjData* d);
VectorXd get_body_pos(const mjModel* m, mjData* d, const char* body_name);
void rotation_matrix_to_euler_angles(const mjtNum* R, double& roll, double& pitch, double& yaw);
void arm_go_home(const mjModel* m, mjData* d);

#endif // FORWARD_KINEMATICS_H
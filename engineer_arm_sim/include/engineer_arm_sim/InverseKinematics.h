#pragma once
#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <stdbool.h>
#include <Eigen/Dense>
#include <vector>
#include "mujoco/mujoco.h"
#include "GLFW/glfw3.h"
// 声明命名空间
using namespace Eigen;
using namespace std;


// 函数声明
MatrixXd JacobianMatrix(const mjModel* m, mjData* d);
MatrixXd pseudo_inverse(const MatrixXd& A);
VectorXd move_ik(VectorXd delta_pos, MatrixXd J_pseudo_inv);
VectorXd inverse_kinematics(const mjModel* m, mjData* d, VectorXd& target_pos);
void endeffector_control_keyboard(GLFWwindow* window);
void endeffector_controller(const mjModel* m, mjData* d);

#endif // INVERSE_KINEMATICS_H
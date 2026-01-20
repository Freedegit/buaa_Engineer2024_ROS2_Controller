#define _CRT_SECURE_NO_WARNINGS
#define delta 3e-3
#define thelta 1e-4
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
#include"engineer_arm_sim/InverseKinematics.h"
using namespace Eigen;
using namespace std;
bool key_up = false;
bool key_down = false;
bool key_left = false;
bool key_right = false;
bool key_forward = false;
bool key_back = false;
bool control = false;
/************************************************************************************
* 在这里定义全局变量。
* 这些全局变量需要在头文件中 InverseKinematics.h 中声明。
* 这些全局变量可以在项目中任何地方调用。
* 但同时需要注意它可能在任何地方被更改。
* 请按照需要增加或者删除。
************************************************************************************/
int CCOUNT = 0;
int N = 50000;
double epsilon = 1e-3;
static VectorXd q_des(6);
static bool inited = false;

MatrixXd JAC(6, 6);
MatrixXd JacobianMatrix(const mjModel* m, mjData* d)
{
	// 计算全局雅可比
	vector<mjtNum> jacp(3 * m->nv, 0);
	vector<mjtNum> jacr(3 * m->nv, 0);
	int panda_link7_body_id = mj_name2id(m, mjOBJ_BODY, "link6");
	mj_jacBody(m, d, jacp.data(), jacr.data(), panda_link7_body_id);

	// link 的 6 个关节名称
	vector<string> joint_names = {
		"joint1","joint2","joint3","joint4","joint5","joint6"
	};

	// 提取每个关节的 qvel 索引
	vector<int> qvel_ids(6);
	for (int i = 0; i < 6; i++) {
		int jid = mj_name2id(m, mjOBJ_JOINT, joint_names[i].c_str());
		qvel_ids[i] = m->jnt_dofadr[jid];
	}

	// 构造 6×6 雅可比矩阵
	MatrixXd J(6, 6);
	for (int j = 0; j < 6; ++j) {
		int id = qvel_ids[j];
		for (int i = 0; i < 3; ++i) {
			J(i, j) = jacp[i * m->nv + id];
			J(i + 3, j) = jacr[i * m->nv + id];
		}
	}
	return J;
}


MatrixXd pseudo_inverse(const MatrixXd& A) 
{
	/* 使用CompleteOrthogonalDecomposition类进行SVD分解以计算矩阵伪逆 */
	CompleteOrthogonalDecomposition<MatrixXd> cod(A);
	MatrixXd pseudo_inverse_A = cod.pseudoInverse();
	return pseudo_inverse_A;
}

void endeffector_control_keyboard(GLFWwindow* window)
{
	/************************************************************************************
	* 1. 本函数将会在main.cpp中以60Hz的频率循环被调用，只需要实现逻辑功能即可。
	* 2. glfwGetKey函数检测是否按住了键盘中的某个按键。
	* 3. 相关按键映射可以参考官方网站：https://www.glfw.org/docs/latest/group__keys.html
	************************************************************************************/
	/************************************************************************************
	* 1. 本函数将会在main.cpp中以60Hz的频率循环被调用，只需要实现逻辑功能即可。
	* 2. glfwGetKey函数检测是否按住了键盘中的某个按键。
	* 3. 相关按键映射可以参考官方网站：https://www.glfw.org/docs/latest/group__keys.html
	************************************************************************************/
	key_up = glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS;  // 键up
	key_down = glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS;  // 键down
	key_forward = glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS;  // 键w
	key_back = glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS;  // 键s
	key_left = glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS;  // 键a
	key_right = glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS;  // 键d
	control = glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS; // 同时检测左右Ctrl键
}

VectorXd keyboard_to_direction()
{
	/*************************************************************************************
	* 参考ForwardKinematics.cpp中的joint_controller函数。
	* 这个函数不直接控制d->ctrl，而是会返回VectorXd形式的6维向量；
	* 注意要将这个向量单位化；
	* 这个函数将被在后续endeffector_controller中被调用。在后续函数中实现赋值。
	*************************************************************************************/
	VectorXd v1=VectorXd::Zero(6);
	if (!control) {
		if (key_up) {
			v1(2) = 1;
		}
		if (key_down) {
			v1(2) = -1;
		}
		if (key_forward) {
			v1(0) = 1;
		}
		if (key_back) {
			v1(0) = -1;
		}
		if (key_left) {
			v1(1) = 1;
		}
		if (key_right) {
			v1(1) = -1;
		}
	}
	else {
		if (key_up) {
			v1(5) = 1;
		}
		if (key_down) {
			v1(5) = -1;
		}
		if (key_forward) {
			v1(3) = 1;
		}
		if (key_back) {
			v1(3) = -1;
		}
		if (key_left) {
			v1(4) = 1;
		}
		if (key_right) {
			v1(4) = -1;
		}
	}
	v1.normalize();
	return v1;
}
void endeffector_controller(const mjModel* m, mjData* d)
{
	/************************************************************************************
	* 1. 本函数将会在main.cpp中以60Hz的频率循环被调用，只需要实现逻辑功能即可。
	* 2. 参考ForwardKinematics.cpp中的joint_controller函数。
	* 3. 调用前面实现的功能，分别计算雅可比矩阵的逆矩阵$$J^*(\theta)$$和夹爪期望的移动方向VectorXd direction
	* 4. 计算$$\Delta\theta = J^* (\theta) * \Delta x$$，其中的$$\Delta x$$是direction在单位化后乘一个足够小的数字。
	* 5. 通过d->ctrl控制机械臂关节运动。
	************************************************************************************/
	if (!inited) {
    for (int i = 0; i < 6; i++) {
        int jid = m->actuator_trnid[2*i];
        int qadr = m->jnt_qposadr[jid];
        q_des(i) = d->qpos[qadr];
    }
    inited = true;
	}
	MatrixXd M(6, 6);
	M= pseudo_inverse(JacobianMatrix(m, d));
	VectorXd v2(6);
	v2= M * (keyboard_to_direction());
	for(int i=0;i<6;i++){
		q_des[i]+=delta*v2(i);
		d->qpos[i]=q_des[i];
	}
}
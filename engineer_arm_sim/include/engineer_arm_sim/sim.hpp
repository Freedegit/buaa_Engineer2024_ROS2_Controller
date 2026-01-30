#pragma once

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

extern mjModel* m ;                  // MuJoCo model
extern mjData* d;                   // MuJoCo data
extern double target_pos[6];
enum class ControllerType {
    PID,
    MIT,
    KINEMATIC
};
extern ControllerType g_controller_type;
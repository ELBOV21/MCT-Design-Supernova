#pragma once
#include <Arduino.h>
#include "kinematics.h"
#include "HiwonderMotorDriver.h"
class MotionController
{
public:
    const double RAMP_STEP = 0.1; // Maximum change in speed per update (m/s or rad/s)
    const double MAX_SPEED = 0.5; // Maximum linear speed (m/s)
    MotionController(HiwonderMotorDriver &motorDriver, Kinematics &kin);

    void setTargetVelocity(double vx, double vy, double omega);
    void stop();
    void update();

private:
    HiwonderMotorDriver &_motorDriver;
    Kinematics &kinematics;
    double vx = 0.0, vy = 0.0, omega = 0.0;                      // Desired velocities (m/s) and angular velocity (rad/s)
    double vx_target = 0.0, vy_target = 0.0, omega_target = 0.0; // Target velocities for control loop

    double ramp(double target, double current, double step);
};
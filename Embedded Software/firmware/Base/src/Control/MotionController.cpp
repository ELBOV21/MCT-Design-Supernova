#include "MotionController.h"

MotionController::MotionController(HiwonderMotorDriver &motorDriver, Kinematics &kin) 
    : _motorDriver(motorDriver), kinematics(kin) {}

void MotionController::setTargetVelocity(double vx, double vy, double omega) {
    // Limit target velocities to MAX_SPEED
    vx_target = constrain(vx, -MAX_SPEED, MAX_SPEED);
    vy_target = constrain(vy, -MAX_SPEED, MAX_SPEED);
    omega_target = constrain(omega, -MAX_SPEED, MAX_SPEED);   
}

double MotionController::ramp(double target, double current, double step) {
    if (current < target) {
        return min(current + step, target);
    } else if (current > target) {
        return max(current - step, target);
    }
    return current; // Already at target
}

void MotionController::stop() {
    vx_target = 0.0;
    vy_target = 0.0;
    omega_target = 0.0;
}

void MotionController::update() {
    // Ramp current velocities towards target velocities
    vx = ramp(vx_target, vx, RAMP_STEP);
    vy = ramp(vy_target, vy, RAMP_STEP);
    omega = ramp(omega_target, omega, RAMP_STEP);

    // Calculate motor speeds using kinematics
    double motor_speeds[4];
    kinematics.calculateMotorSpeeds(vx, vy, omega, motor_speeds);

    // Convert motor speeds from rad/s to the scale expected by the motor driver (e.g., -100 to 100)
    int8_t speedCommands[4];
    for (int i = 0; i < 4; i++) {
        // Assuming max wheel speed corresponds to MAX_SPEED in m/s
        // and that the motor driver expects values in the range -100 to 100
        speedCommands[i] = (int8_t)constrain((motor_speeds[i] / (MAX_SPEED / kinematics.getWheelRadius())) * 100.0, -100.0, 100.0);
    }

    // Send speed commands to the motor driver
    _motorDriver.setSpeed(speedCommands);
}
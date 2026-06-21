#include "Kinematics.h"

Kinematics::Kinematics(float w_r, float l_x , float l_y ) {
        wheel_radius = w_r;
        lx = l_x;
        ly = l_y;
    }

void Kinematics::calculateMotorSpeeds(float linear_x, float linear_y, float angular_z, double* motor_speeds) {
    // The Standard Robotics Coordinate Frame:
    // linear_x: Forward velocity (m/s)       --> +X is Forward
    // linear_y: Strafe velocity (m/s)        --> +Y is Left
    // angular_z: Rotational velocity (rad/s) --> +W is Counter-Clockwise (Left Twist)

    float geometry_factor = lx + ly;

    // Calculate expected physical wheel rotational velocities in rad/s.
    // A positive output here means the wheel must physically roll FORWARD.
    float speed_FR = (linear_x + linear_y + (angular_z * geometry_factor)) / wheel_radius;
    float speed_FL = (linear_x - linear_y - (angular_z * geometry_factor)) / wheel_radius;
    float speed_BR = (linear_x - linear_y + (angular_z * geometry_factor)) / wheel_radius;
    float speed_BL = (linear_x + linear_y - (angular_z * geometry_factor)) / wheel_radius;

    // Map to your specific array index format: 
    // [0]=Front-Right, [1]=Front-Left, [2]=Back-Right, [3]=Back-Left
    motor_speeds[0] = speed_FR;
    motor_speeds[1] = speed_FL;
    motor_speeds[2] = speed_BR;
    motor_speeds[3] = speed_BL;
}

float Kinematics::getWheelRadius() {
    return wheel_radius;
}
#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>

class Kinematics {
private:
    float wheel_radius;
    float lx; // Distance from robot center to the front axle (meters)
    float ly; // Distance from robot center to the left wheels (meters)

public:
    Kinematics(float w_r = 0.04, float l_x = 0.1, float l_y = 0.17);
    void calculateMotorSpeeds(float linear_x, float linear_y, float angular_z, double* motor_speeds);
    float getWheelRadius();
};

#endif
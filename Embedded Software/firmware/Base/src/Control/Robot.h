#pragma once
#include <Arduino.h>
#include "MotionController.h"
#include "HiwonderMotorDriver.h"

class Robot
{
public:
    Robot(MotionController &motion, HiwonderMotorDriver &motorDriver);

    void update();

    void moveForward(double distance_cm);
    void moveBackward(double distance_cm);
    void strafeLeft(double distance_cm);
    void strafeRight(double distance_cm);
    void rotateLeft(double angle_deg);
    void rotateRight(double angle_deg);
    void stop();

private:
    MotionController &motionController;
    HiwonderMotorDriver &motorDriver;

    // Helper functions for distance and angle conversions
    double cmToMeters(double cm) { return cm / 100.0; }
    double degToRad(double deg) { return deg * (PI / 180.0); }

    const double WHEEL_RADIUS = 0.04; // meters
    const double GEAR_RATIO = 34.0;    // Gear Ratio
    const double TICKS_PER_REVOLUTION = 11.0 * GEAR_RATIO; // ticks 

};
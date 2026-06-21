#ifndef MANUAL_CONTROL_H
#define MANUAL_CONTROL_H

#include <Arduino.h>

typedef struct ESPNOW_Message {
    int command;
    int transX;        // Replaced value1
    int transY;        // Replaced value2
    int rotZ;          // Replaced value3
    bool isAutonomous; // The new toggle switch state
} ESPNOW_Message;

struct TargetVelocity {
    float vx;    // Forward velocity (m/s)
    float vy;    // Strafe velocity (m/s) -> +Y is Left in your Kinematics
    float omega; // Rotational velocity (rad/s)
};

class ManualControl {
private:
    float maxLinearVel;
    float maxAngularVel;
    int centerVal;
    int deadband;

public:
    // Constructor
    ManualControl(float maxLin, float maxAng, int center = 2048, int deadzone = 300);
    
    // Main processing function
    TargetVelocity processInput(const ESPNOW_Message& msg);
};

#endif
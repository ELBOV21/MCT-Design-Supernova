#include "manual_control.h"
#include <math.h>

ManualControl::ManualControl(float maxLin, float maxAng, int center, int deadzone) {
    maxLinearVel = maxLin;
    maxAngularVel = maxAng;
    centerVal = center; // We will pass 1700 into this from the main file
    deadband = deadzone;
}

TargetVelocity ManualControl::processInput(const ESPNOW_Message& msg) {
    TargetVelocity target = {0.0, 0.0, 0.0};

    // 1. Calculate raw distance from center
    int dx_raw = msg.transX - centerVal; // X-axis (Left/Right)
    int dy_raw = msg.transY - centerVal; // Y-axis (Forward/Backward)
    int dr_raw = msg.rotZ - centerVal; // Z-axis (Rotation)

    // 2. Normalize to a perfect -1.0 to 1.0 range to fix the hardware asymmetry
    float normX = 0, normY = 0, normW = 0;

    // Normalize X (Left=0 -> normX=-1.0, Right=4095 -> normX=1.0)
    if (dx_raw < 0) normX = (float)dx_raw / (float)centerVal;
    else normX = (float)dx_raw / (float)(4095 - centerVal);

    // Normalize Y (Forward=0 -> normY=-1.0, Backward=4095 -> normY=1.0)
    if (dy_raw < 0) normY = (float)dy_raw / (float)centerVal;
    else normY = (float)dy_raw / (float)(4095 - centerVal);

    // Normalize Rotation (Assuming Left=0, Right=4095)
    if (dr_raw < 0) normW = (float)dr_raw / (float)centerVal;
    else normW = (float)dr_raw / (float)(4095 - centerVal);

    // Convert raw deadband to a normalized threshold (e.g., 200 / 1700 approx 0.12)
    float normDeadband = (float)deadband / (float)centerVal;

    // ---------------------------------------------------------
    // MODE 0: Constant Velocity (8 Directions)
    // ---------------------------------------------------------
    if (msg.command == 2) {
        
        // Forward/Backward (normY is negative when pushed Forward)
        if (normY < -normDeadband) target.vx = maxLinearVel;       // Forward
        else if (normY > normDeadband) target.vx = -maxLinearVel;  // Backward

        // Left/Right (normX is negative when pushed Left)
        if (normX < -normDeadband) target.vy = maxLinearVel;       // Left
        else if (normX > normDeadband) target.vy = -maxLinearVel;  // Right

        // Rotation (normW negative usually means twist left/CCW)
        if (normW < -normDeadband) target.omega = maxAngularVel;       // Turn Left (CCW)
        else if (normW > normDeadband) target.omega = -maxAngularVel;  // Turn Right (CW)
    } 
    
    // ---------------------------------------------------------
    // MODE 1: Continuous Velocity (Radial Threshold)
    // ---------------------------------------------------------
    else if (msg.command == 3) {
        
        // Calculate the vector magnitude using the normalized values
        float r = sqrt((normX * normX) + (normY * normY));

        // Only move if the stick is pushed past the radial deadzone
        if (r > normDeadband) {
            
            // Map the velocities. We invert the sign (-) so that pushing
            // to 0 (which gave a negative norm) results in a positive velocity.
            target.vx = -normY * maxLinearVel;
            target.vy = -normX * maxLinearVel;
            
            // Safety Clamp
            if (target.vx > maxLinearVel) target.vx = maxLinearVel;
            if (target.vx < -maxLinearVel) target.vx = -maxLinearVel;
            if (target.vy > maxLinearVel) target.vy = maxLinearVel;
            if (target.vy < -maxLinearVel) target.vy = -maxLinearVel;
        }

        // Handle rotation independently of X/Y translation
        if (abs(normW) > normDeadband) {
            target.omega = -normW * maxAngularVel;
        }
    }

    return target;
}
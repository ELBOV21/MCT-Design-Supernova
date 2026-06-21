 #include "TrajectoryPlanning.h"
#include "../Drivers/Servo_Driver.h"
#include <Arduino.h>
#include "ControllerToArm.h"
extern void BroadcastArmTelemetry();
extern int joint_angles[];
// ---------------------------------------------------------------------------
// Pose table
//
// Edit only the angle values; do NOT change the enum labels or array order
// (they must stay in sync with the PoseID enum in TrajectoryPlanning.h).
//
// Format: { base, shoulder, elbow, wrist }  — all in degrees
// ---------------------------------------------------------------------------
Pose poses[NUM_POSES] = {
    [HOME]          = {{ 88,  60,  10, 180 }},
    [PICK]          = {{ 90,  92, 175, 180 }},
    [GRIP_POSE]     = {{ 90,  92, 175,  87 }},
    [RIGHT]         = {{ 60,  84,   0, 178 }},
    [RIGHT_UP]      = {{ 60,  60,   0, 178 }},
    [RIGHT_PLACE]   = {{ 60,  65,   0, 178 }},
    [LEFT]          = {{120,  80,   0, 175 }},
    [LEFT_UP]       = {{120,  60,  10, 180 }},
    [LEFT_PLACE]    = {{120,  65,   0, 175 }},
    [DROP]          = {{ 90,  40, 180,  50 }},
    [MIDDLE]        = {{ 88, 113,  40, 178 }},
    [MIDDLE_UP]     = {{ 88, 105,  45, 178 }},
    [MIDDLE_PLACE]  = {{ 90, 105,  45, 178 }},
    [CAPTURE]       = {{ 81,  80, 170,  90 }},
    [PLACEONPEDESTAL] = {{  0,   0,   0,   0 }},
};


// ---------------------------------------------------------------------------
// Trajectory math
// ---------------------------------------------------------------------------

float quinticTrajectory(float t, float T, float q0, float qf)
{
    float s = t / T;
    if (s > 1.0f) s = 1.0f;
    // 5th-order polynomial: zero velocity & acceleration at both endpoints
    float blend = 10.0f * powf(s, 3) - 15.0f * powf(s, 4) + 6.0f * powf(s, 5);
    return q0 + (qf - q0) * blend;
}


// ---------------------------------------------------------------------------
// Motion primitives
// ---------------------------------------------------------------------------

void movePose(PoseID from, PoseID to, float T)
{
    unsigned long startTime = millis();

    while (true)
    {
        float t = (millis() - startTime) / 1000.0f;

        for (int i = 0; i < CFG_NUM_JOINTS; ++i)
        {
            float angle = quinticTrajectory(t, T, poses[from].angles[i], poses[to].angles[i]);
            Servo_SetAngle(i, angle);
            joint_angles[i] = static_cast<int>(angle); 
        }
        BroadcastArmTelemetry(); 
        if (t >= T) break;
        delay(20);
    }

    // Guarantee exact final position (eliminates floating-point drift)
    for (int i = 0; i < CFG_NUM_JOINTS; ++i)
    {
        Servo_SetAngle(i, poses[to].angles[i]);
        joint_angles[i] = static_cast<int>(poses[to].angles[i]);
    }
}

void moveFromCurrentPoseToTargetPose(PoseID to, float T)
{
    // Snapshot current angles as the start of this motion
    float startAngles[CFG_NUM_JOINTS];
    for (int i = 0; i < CFG_NUM_JOINTS; ++i)
    {
        startAngles[i] = static_cast<float>(joint_angles[i]);
    }

    unsigned long startTime = millis();

    while (true)
    {
        float t = (millis() - startTime) / 1000.0f;

        for (int i = 0; i < CFG_NUM_JOINTS; ++i)
        {
            float angle = quinticTrajectory(t, T, startAngles[i], poses[to].angles[i]);
            joint_angles[i] = static_cast<int>(angle);
            Servo_SetAngle(i, angle);
        }

        if (t >= T) break;
        delay(20);
    }

    // Guarantee exact final position
    for (int i = 0; i < CFG_NUM_JOINTS; ++i)
    {
        Servo_SetAngle(i, poses[to].angles[i]);
        joint_angles[i] = static_cast<int>(poses[to].angles[i]);
    }
}
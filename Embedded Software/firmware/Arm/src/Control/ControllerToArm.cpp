#include "ControllerToArm.h"
#include "../Drivers/Servo_Driver.h"

// ---------------------------------------------------------------------------
// Global joint-angle state
// Index 0-3 = arm joints, index 4 = gripper
// ---------------------------------------------------------------------------
int joint_angles[CFG_NUM_JOINTS + 1] = {
    CFG_JOINT1_INIT_ANGLE,
    CFG_JOINT2_INIT_ANGLE,
    CFG_JOINT3_INIT_ANGLE,
    CFG_JOINT4_INIT_ANGLE,
    CFG_GRIPPER_INIT_ANGLE
};


// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/**
 * @brief Convert a raw joystick ADC reading to a directional step (-1, 0, +1).
 *
 * Values below CFG_JOY_LOW_THRESHOLD  → -1 
 * Values above CFG_JOY_HIGH_THRESHOLD → +1 
 * Values in between                   →  0 
 */
static int joystickToAngle(int rawValue)
{
    if (rawValue < CFG_JOY_LOW_THRESHOLD)  return -1;
    if (rawValue > CFG_JOY_HIGH_THRESHOLD) return  1;
    return 0;
}

/**
 * @brief Clamp an angle to the configured joint limits.
 */
static int clampAngle(int angle)
{
    if (angle > CFG_JOINT_ANGLE_MAX) return CFG_JOINT_ANGLE_MAX;
    if (angle < CFG_JOINT_ANGLE_MIN) return CFG_JOINT_ANGLE_MIN;
    return angle;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void servo_angles_init(int angles[])
{
    for (int i = 0; i <= CFG_NUM_JOINTS; ++i)
    {
        Servo_SetAngle(i, angles[i]);
    }
}


void set_joint_angles(const ControllerToArm &controller_data, ArmToController &armdata)
{
    // Map joystick channels to joint indices
    const int rawValues[CFG_NUM_JOINTS] = {
        controller_data.joint1,
        controller_data.joint2,
        controller_data.joint3,
        controller_data.joint4
    };

    for (int i = 0; i < CFG_NUM_JOINTS; ++i)
    {
        joint_angles[i] = clampAngle(joint_angles[i] + joystickToAngle(rawValues[i]));
        Servo_SetAngle(i, joint_angles[i]);

        Serial.print("Joint ");
        Serial.print(i + 1);
        Serial.print(" -> ");
        Serial.print(joint_angles[i]);
        Serial.println("°");
    }

    // Populate telemetry reply
    armdata.joint0 = joint_angles[0];
    armdata.joint1 = joint_angles[1];
    armdata.joint2 = joint_angles[2];
    armdata.joint3 = joint_angles[3];
}

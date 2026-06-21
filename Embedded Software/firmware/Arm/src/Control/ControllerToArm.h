#ifndef CONTROLLER_TO_ARM_H
#define CONTROLLER_TO_ARM_H

#include <Arduino.h>
#include "Config/Config.h"


/**
 * @brief Packet sent FROM the handheld controller TO the arm.
 */
struct ControllerToArm {
    int  joint1;               ///< Raw joystick ADC value for joint 1 (base)
    int  joint2;               ///< Raw joystick ADC value for joint 2 (shoulder)
    int  joint3;               ///< Raw joystick ADC value for joint 3 (elbow)
    int  joint4;               ///< Raw joystick ADC value for joint 4 (wrist)
    bool Grip;                 ///< true = close gripper
    int  armPositionCommand;   ///< Preset position command (1–7) 
    bool ArmSpeed;             ///< true = fast mode
    bool isAutonomous;         ///< true = autonomous mode active 
};

/**
 * @brief Packet sent FROM the arm BACK to the controller
 */
struct ArmToController {
    int joint0;  ///< Current angle of joint 1 (degrees)
    int joint1;  ///< Current angle of joint 2 (degrees)
    int joint2;  ///< Current angle of joint 3 (degrees)
    int joint3;  ///< Current angle of joint 4 (degrees)
};

// ---------------------------------------------------------------------------
// Global joint-angle state  (defined in ControllerToArm.cpp)
// ---------------------------------------------------------------------------
extern int joint_angles[CFG_NUM_JOINTS + 1];  ///< Indices 0-3 = joints, 4 = gripper

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/**
 * @brief Drive all servos to their initial angles.
 * @param angles  Array of (CFG_NUM_JOINTS + 1) initial angles.
 */
void servo_angles_init(int angles[]);

/**
 * @brief Process incoming joystick packet and update joint angles.
 * @param controller_data  Incoming controller packet.
 * @param armdata          Telemetry struct to be populated.
 */
void set_joint_angles(const ControllerToArm &controller_data, ArmToController &armdata);



#endif // CONTROLLER_TO_ARM_H
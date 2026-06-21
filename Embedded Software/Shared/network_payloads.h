#ifndef NETWORK_PAYLOADS_H
#define NETWORK_PAYLOADS_H

#include <Arduino.h>

// =========================================================================
// 1. Inbound Command (HMI Controller -> Mobile Base)
// =========================================================================
typedef struct ESPNOW_Message {
    int command;       // 2=Velocity, 3=Continuous, 4=Auto350, 5=Auto200
    int transX;        // Joystick X-axis (0-4095)
    int transY;        // Joystick Y-axis (0-4095)
    int rotZ;          // Joystick Z-axis (0-4095)
    bool isAutonomous; // Triggers transition to AutonomousMission
} ESPNOW_Message;

// =========================================================================
// 2. Direct Arm Control (HMI Controller -> Manipulator Arm)
// =========================================================================
struct ControllerToArm {
    int joint1; int joint2; int joint3; int joint4; // Raw Joystick telemetry
    bool Grip;               // True = Close Gripper
    int armPositionCommand;  // Pre-programmed spatial array index (1-7)
    bool ArmSpeed;           // True = Fast Mode (25ms), False = Slow (100ms)
    bool isAutonomous;       // Sequence trigger flag
};

// =========================================================================
// 3. Sequence Synchronization (Mobile Base <-> Manipulator Arm)
// =========================================================================
struct BaseArmSync {
    int auto_command; // Base sends 1, 3, 5 (Drop Commands). Arm returns 2, 4, 6 (ACKs).
};

// =========================================================================
// 4. Live Arm Telemetry (Manipulator Arm -> Mobile Base)
// =========================================================================
struct ArmTelemetryToCar {
    int joint1; int joint2; int joint3; int joint4; // Current physical angles
    int gripper;                                    // Current gripper angle
};

// =========================================================================
// 5. Sensor Feedback (Mobile Base -> HMI Controller)
// =========================================================================
typedef struct CarToController {
    int tofDistance_mm; // Measured distance from VL53L0X sensor
} CarToController;

// =========================================================================
// 6. Camera Triggers & Results (Controller -> Camera -> Base)
// =========================================================================
typedef struct {
    char payload[128]; // Contains "START", "TIMEOUT", or color strings
} espnow_msg_t;

// =========================================================================
// 7. Digital Twin Telemetry Fusion (Mobile Base -> PC GUI)
// =========================================================================
struct GUI_par {
    float roll; float yaw; float pitch;             // IMU Data
    float encoder1; float encoder2;                 // Motor Odometry
    float encoder3; float encoder4;
    float vx; float vy; float omega;                // Forward Kinematics
    float x; float y;                               // Global Integrated Position
    char vehicleState[32];                          // System Status
    int tofDistance;                                // VL53L0X Data
    float joint1_angle; float joint2_angle;         // Relayed Arm Telemetry
    float joint3_angle; float joint4_angle;
    float gripper_val;
};

#endif
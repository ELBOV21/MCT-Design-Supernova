#pragma once

// =============================================================================
// Config.h  —  Central configuration for the Robotic Arm project
//
// Edit this file to tune hardware parameters, network addresses, and motion
// presets without touching any other source file.
// =============================================================================


// ---------------------------------------------------------------------------
// Network  (ESP-NOW MAC addresses)
// ---------------------------------------------------------------------------
// Format: { 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX }
#define CFG_MAC_CONTROLLER  { 0x44, 0x1D, 0x64, 0xF5, 0xFD, 0xB0 }
#define CFG_MAC_BASE_CAR    { 0x80, 0xF3, 0xDA, 0x54, 0xAC, 0x58 }

// ---------------------------------------------------------------------------
// Serial
// ---------------------------------------------------------------------------
#define CFG_SERIAL_BAUD     115200
// ---------------------------------------------------------------------------
// Servo hardware  (PCA9685 / Adafruit PWM driver)
// ---------------------------------------------------------------------------
#define CFG_PWM_OSCILLATOR_HZ   25000000UL  // Internal oscillator frequency
#define CFG_PWM_FREQ_HZ         50          // Standard servo PWM frequency

// Pulse-width range in microseconds for 0°–180°
#define CFG_SERVO_PULSE_MIN_US  500
#define CFG_SERVO_PULSE_MAX_US  2500

// Number of arm joints (excluding gripper)
#define CFG_NUM_JOINTS          4

// ---------------------------------------------------------------------------
// Joint indices
// ---------------------------------------------------------------------------
#define CFG_JOINT_0      0
#define CFG_JOINT_1      1
#define CFG_JOINT_2      2
#define CFG_JOINT_3      3
#define CFG_JOINT_GRIPPER   4

// ---------------------------------------------------------------------------
// Joint angle limits  (degrees, applied to all joints uniformly)
// ---------------------------------------------------------------------------
#define CFG_JOINT_ANGLE_MIN     0
#define CFG_JOINT_ANGLE_MAX     180

// ---------------------------------------------------------------------------
// Joint initial / home angles  (degrees)
// ---------------------------------------------------------------------------
#define CFG_JOINT1_INIT_ANGLE   80
#define CFG_JOINT2_INIT_ANGLE   60
#define CFG_JOINT3_INIT_ANGLE   10
#define CFG_JOINT4_INIT_ANGLE   180
#define CFG_GRIPPER_INIT_ANGLE  180

// ---------------------------------------------------------------------------
// Gripper angles
// ---------------------------------------------------------------------------
#define CFG_GRIP_ANGLE_CLOSED   60   // Angle when gripping an object
#define CFG_GRIP_ANGLE_OPEN     180  // Angle when open / released

// ---------------------------------------------------------------------------
// Control loop speeds  (milliseconds between joint-update cycles)
// ---------------------------------------------------------------------------
#define CFG_SPEED_FAST_MS   25   // High-speed mode  (ArmSpeed = true)
#define CFG_SPEED_SLOW_MS   100  // Low-speed  mode  (ArmSpeed = false)

// ---------------------------------------------------------------------------
// Joystick dead-zone thresholds  (ADC counts, 0–4095)
// ---------------------------------------------------------------------------
#define CFG_JOY_LOW_THRESHOLD   1000  // Below this → direction = -1
#define CFG_JOY_HIGH_THRESHOLD  3000  // Above this → direction = +1

// ---------------------------------------------------------------------------
// ToF sensor (VL53L0X) distances  (millimetres)
// ---------------------------------------------------------------------------
#define CFG_TOF_CAPTURE_DISTANCE_MM  350
#define CFG_TOF_GRIP_DISTANCE_MM     225

// ---------------------------------------------------------------------------
// Trajectory motion durations  (seconds, used as defaults in motion sequences)
// ---------------------------------------------------------------------------
#define CFG_MOTION_FAST_S   2.0f
#define CFG_MOTION_SLOW_S   4.0f
#define CFG_MOTION_PLACE_S  6.0f

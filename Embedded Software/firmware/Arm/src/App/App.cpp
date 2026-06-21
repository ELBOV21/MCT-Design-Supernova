// =============================================================================
// App.cpp  —  Application layer implementation
//
// Owns all firmware logic:
//   • ESP-NOW initialisation and receive callback
//   • Autonomous pick-and-place sequences
//   • Gripper and continuous joint control
// =============================================================================

#include "App/App.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include "Config/Config.h"
#include "Control/ControllerToArm.h"
#include "Control/TrajectoryPlanning.h"
#include "Drivers/Servo_Driver.h"

// ---------------------------------------------------------------------------
// MAC addresses (from Config.h)
// ---------------------------------------------------------------------------
static uint8_t s_controllerAddr[] = CFG_MAC_CONTROLLER;
static uint8_t s_carAddr[]        = CFG_MAC_BASE_CAR;

// ---------------------------------------------------------------------------
// Wire-format sync packet shared with the base car
// ---------------------------------------------------------------------------
struct BaseArmSync {
    int auto_command;  ///< Command code sent by car / ACK code sent by arm
};

struct ArmTelemetryToCar {
    int joint1;
    int joint2;
    int joint3;
    int joint4;
    int gripper;
};

// ---------------------------------------------------------------------------
// Application state
// ---------------------------------------------------------------------------
static ControllerToArm s_controllerData  = {};
static ArmToController  s_armData        = {};
static bool             s_dataReady      = false;
static int              s_pendingAutoCmd = 0;
static unsigned long    s_lastUpdateMs   = 0;
extern int joint_angles[]; // Bring in the global array from ControllerToArm.cpp

static ArmTelemetryToCar s_carTelemetry = {};

// ===========================================================================
// ESP-NOW
// ===========================================================================
void sendAckToCar(int ack_code)
{
    BaseArmSync ackMsg;
    ackMsg.auto_command = ack_code;
    esp_now_send(s_carAddr, (uint8_t *)&ackMsg, sizeof(ackMsg));
}

void BroadcastArmTelemetry() {
    s_carTelemetry.joint1 = joint_angles[0];
    s_carTelemetry.joint2 = joint_angles[1];
    s_carTelemetry.joint3 = joint_angles[2];
    s_carTelemetry.joint4 = joint_angles[3];
    s_carTelemetry.gripper = joint_angles[4]; // Index 4 is the end-effector
    Serial.print("Broadcasting Telemetry: ");
    Serial.print("J1: "); Serial.print(s_carTelemetry.joint1);
    Serial.print(", J2: "); Serial.print(s_carTelemetry.joint2);
    Serial.print(", J3: "); Serial.print(s_carTelemetry.joint3);
    Serial.print(", J4: "); Serial.print(s_carTelemetry.joint4);
    Serial.print(", Gripper: "); Serial.print(s_carTelemetry.gripper);
    Serial.println();
    esp_now_send(s_carAddr, (uint8_t *)&s_carTelemetry, sizeof(s_carTelemetry));
}

void SetGripperState(int angle) {
    Servo_SetAngle(CFG_JOINT_GRIPPER, angle);
    joint_angles[4] = angle;
    BroadcastArmTelemetry(); // Fire an immediate update to the GUI
}

// --- ESP-NOW Receive Callback ---
void OnDataReceive(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    if (len == sizeof(ControllerToArm))
    {
        memcpy(&s_controllerData, incomingData, sizeof(s_controllerData));
        s_dataReady = true;
    }
    else if (len == sizeof(BaseArmSync))
    {
        BaseArmSync syncMsg;
        memcpy(&syncMsg, incomingData, sizeof(syncMsg));
        s_pendingAutoCmd = syncMsg.auto_command;
    }
}

static bool initEspNow()
{
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("[ERROR] ESP-NOW init failed");
        return false;
    }

    esp_now_peer_info_t peerInfo = {};
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    memcpy(peerInfo.peer_addr, s_controllerAddr, 6);
    esp_now_add_peer(&peerInfo);

    memcpy(peerInfo.peer_addr, s_carAddr, 6);
    esp_now_add_peer(&peerInfo);

    esp_now_register_recv_cb(OnDataReceive);
    return true;
}

// ===========================================================================
// Autonomous pick-and-place sequences
// ===========================================================================

/**
 * @brief Pick the Red box and place it in the LEFT bin.  auto_command 1 → ACK 2.
 */
static void placeBoxRed()
{
    Serial.println("[AUTO] Place box RED");
    SetGripperState(CFG_GRIP_ANGLE_OPEN); // FIXED: Removed redundant joint ID
    movePose(HOME,    LEFT_UP,   CFG_MOTION_SLOW_S);
    movePose(LEFT_UP, LEFT,      CFG_MOTION_FAST_S);
    SetGripperState(CFG_GRIP_ANGLE_CLOSED); // FIXED
    movePose(LEFT,    LEFT_UP,   CFG_MOTION_FAST_S);
    movePose(LEFT_UP, DROP,      CFG_MOTION_SLOW_S);
    SetGripperState(CFG_GRIP_ANGLE_OPEN); // FIXED
    movePose(DROP,    MIDDLE_UP, CFG_MOTION_SLOW_S);
}

/**
 * @brief Pick the Green box and place it in the MIDDLE bin.  auto_command 3 → ACK 4.
 */
static void placeBoxGreen()
{
    Serial.println("[AUTO] Place box GREEN");
    movePose(MIDDLE_UP, MIDDLE,    CFG_MOTION_FAST_S);
    SetGripperState(CFG_GRIP_ANGLE_CLOSED); // FIXED
    movePose(MIDDLE,    MIDDLE_UP, CFG_MOTION_FAST_S);
    movePose(MIDDLE_UP, DROP,      CFG_MOTION_SLOW_S);
    SetGripperState(CFG_GRIP_ANGLE_OPEN); // FIXED
    movePose(DROP,      RIGHT_UP,  CFG_MOTION_SLOW_S);
}

/**
 * @brief Pick the Blue box and place it in the RIGHT bin.  auto_command 5 → ACK 6.
 */
static void placeBoxBlue()
{
    Serial.println("[AUTO] Place box BLUE");
    movePose(RIGHT_UP, RIGHT,    CFG_MOTION_FAST_S);
    SetGripperState(CFG_GRIP_ANGLE_CLOSED); // FIXED
    movePose(RIGHT,    RIGHT_UP, CFG_MOTION_FAST_S);
    movePose(RIGHT_UP, DROP,     CFG_MOTION_SLOW_S);
    SetGripperState(CFG_GRIP_ANGLE_OPEN); // FIXED
    movePose(DROP,     HOME,     CFG_MOTION_SLOW_S);
}

// ===========================================================================
// Manual preset positions
// ===========================================================================

static void goHome()
{
    Serial.println("[ARM] -> HOME");
    moveFromCurrentPoseToTargetPose(HOME, CFG_MOTION_SLOW_S);
}

static void goLeftPlace()
{
    Serial.println("[ARM] -> LEFT PLACE (red)");
    moveFromCurrentPoseToTargetPose(LEFT_PLACE, CFG_MOTION_PLACE_S);
}

static void goMiddlePlace()
{
    Serial.println("[ARM] -> MIDDLE PLACE (green)");
    moveFromCurrentPoseToTargetPose(MIDDLE_PLACE, CFG_MOTION_PLACE_S);
}

static void goRightPlace()
{
    Serial.println("[ARM] -> RIGHT PLACE (blue)");
    moveFromCurrentPoseToTargetPose(RIGHT_PLACE, CFG_MOTION_PLACE_S);
}

static void goCapture()
{
    Serial.println("[ARM] -> CAPTURE");
    moveFromCurrentPoseToTargetPose(CAPTURE, CFG_MOTION_SLOW_S);
}

static void goGripPreset()
{
    Serial.println("[ARM] -> GRIP PRESET");
    moveFromCurrentPoseToTargetPose(GRIP_POSE, CFG_MOTION_SLOW_S);
}

static void goPick()
{
    Serial.println("[ARM] -> PICK");
    moveFromCurrentPoseToTargetPose(PICK, CFG_MOTION_SLOW_S);
}

// ===========================================================================
// Internal — Dispatch helpers
// ===========================================================================

static void handleAutonomousCommand(int cmd)
{
    switch (cmd)
    {
        case 1: placeBoxRed();   s_pendingAutoCmd = 0;sendAckToCar(2); break;
        case 3: placeBoxGreen(); s_pendingAutoCmd = 0;sendAckToCar(4); break;
        case 5: placeBoxBlue();  s_pendingAutoCmd = 0;sendAckToCar(6); break;
        default:
            Serial.print("[WARN] Unknown auto command: ");
            Serial.println(cmd);
            break;
    }
}

static void handleManualPreset(int cmd)
{
    switch (cmd)
    {
        case 1: goHome();       break;
        case 2: goLeftPlace();  break;
        case 3: goMiddlePlace(); break;
        case 4: goRightPlace(); break;
        case 5: goCapture();    break;
        case 6: goGripPreset(); break;
        case 7: goPick();       break;
        default:
            Serial.print("[WARN] Unknown preset command: ");
            Serial.println(cmd);
            break;
    }
}

// ===========================================================================
// Public API
// ===========================================================================

void App_Init()
{
    Serial.begin(CFG_SERIAL_BAUD);
    Serial.println("[ARM] Booting...");

    if (!initEspNow())
    {
        Serial.println("[ARM] Halted due to ESP-NOW failure.");
        while (true) { delay(1000); }
    }

    Servo_Init();
    servo_angles_init(joint_angles);

    s_lastUpdateMs = millis();
    Serial.println("[ARM] Ready.");
}


void App_Run()
{
    // ----------------------------------------------------------------
    // Priority 1 — Autonomous commands from the base car
    // ----------------------------------------------------------------
    if (s_pendingAutoCmd != 0)
    {
        int cmd      = s_pendingAutoCmd;
        s_pendingAutoCmd = 0;          // Clear before executing
        handleAutonomousCommand(cmd);
        return;                        // Skip manual processing this cycle
    }

    // ----------------------------------------------------------------
    // Priority 2 — Manual preset commands from the OLED menu
    // ----------------------------------------------------------------
    if (!s_controllerData.isAutonomous && s_controllerData.armPositionCommand > 0)
    {
        int cmd = s_controllerData.armPositionCommand;
        s_controllerData.armPositionCommand = 0;   // Consume
        Serial.print("[MANUAL] Preset: ");
        Serial.println(cmd);
        handleManualPreset(cmd);
    }

    // ----------------------------------------------------------------
    // Priority 3 — Gripper (every cycle, not rate-limited)
    // ----------------------------------------------------------------
    int grip_cmd = s_controllerData.Grip ? CFG_GRIP_ANGLE_CLOSED : CFG_GRIP_ANGLE_OPEN;
    if (joint_angles[4] != grip_cmd) {
        SetGripperState(grip_cmd);
    }

    // ----------------------------------------------------------------
    // Priority 4 — Continuous joint control (rate-limited)
    // ----------------------------------------------------------------
    unsigned long now      = millis();
    unsigned long interval = s_controllerData.ArmSpeed ? CFG_SPEED_FAST_MS : CFG_SPEED_SLOW_MS;

    if ((now - s_lastUpdateMs) >= interval && s_dataReady)
    {
        s_dataReady    = false;
        s_lastUpdateMs = now;

        set_joint_angles(s_controllerData, s_armData);
        esp_now_send(s_controllerAddr,
                     (uint8_t *)(&s_armData),
                     sizeof(s_armData));
    }

    // ----------------------------------------------------------------
    // Priority 5 — Continuous Telemetry to Base Car (20Hz)
    // ----------------------------------------------------------------
    static unsigned long s_lastTelemetryMs = 0;
    if (now - s_lastTelemetryMs >= 50)
    {
        s_lastTelemetryMs = now;
        BroadcastArmTelemetry();
    }
}
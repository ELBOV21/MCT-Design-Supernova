#include "mission_manager.h"
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Wire.h>
#include "GUI.h"
#include "manual_control.h"
#include "Kinematics.h"
#include "HiwonderMotorDriver.h"
#include "tof.h" 
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

TaskHandle_t tofTaskHandle = NULL;
SemaphoreHandle_t i2c_mutex;
extern GUI_par LiveValues; // Access the GUI struct from GUI.cpp

// --- MAC Addresses ---
static uint8_t controllerAddress[] = {0x44, 0x1D, 0x64, 0xF5, 0xFD, 0xB0};
static uint8_t armAddress[] = {0x80, 0xF3, 0xDA, 0x54, 0x29, 0x1C};

// --- Structs ---
typedef struct BaseArmSync { int auto_command; } BaseArmSync;
typedef struct CarToController { int tofDistance_mm; } CarToController;
struct ArmTelemetryToCar {
    int joint1;
    int joint2;
    int joint3;
    int joint4;
    int gripper;
};
// --- Global Variables ---
volatile int arm_ack_state = 0;
volatile int current_tof_distance_mm = -1; // Shared distance for Auto-Center
static CarToController outgoingTelemetry;
static TOFSensor tofSensor;

static ManualControl manualDrive(0.2, 0.5, 1850, 300);
static Kinematics kinematics(0.04, 0.1, 0.17);
static HiwonderMotorDriver motorDriver;

static ESPNOW_Message incomingMsg;
static volatile bool newDataReady = false;

const double RADS_TO_RPM = -9.54929659643;
volatile bool is_autonomous_mode = false;
volatile bool trigger_auto_sequence = false;

// --- Callbacks & Helpers ---
void send_arm_command(int cmd) {
    BaseArmSync syncMsg;
    syncMsg.auto_command = cmd;
    esp_now_send(armAddress, (uint8_t *)&syncMsg, sizeof(syncMsg));
}

static void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    // 1. Identify the sender by MAC Address
    bool isFromArm = true;
    bool isFromController = true;
    
    for (int i = 0; i < 6; i++) {
        if (mac[i] != armAddress[i]) isFromArm = false;
        if (mac[i] != controllerAddress[i]) isFromController = false;
    }

    // 2. Route Controller Packets
    if (isFromController && len == sizeof(ESPNOW_Message)) {
        memcpy((void *)&incomingMsg, incomingData, sizeof(incomingMsg));
        newDataReady = true;
    } 
    // 3. Route Arm Packets
    else if (isFromArm) {
        if (len == sizeof(BaseArmSync)) {
            BaseArmSync syncMsg;
            memcpy(&syncMsg, incomingData, sizeof(syncMsg));
            arm_ack_state = syncMsg.auto_command;
        }
        else if (len == sizeof(ArmTelemetryToCar)) {
            ArmTelemetryToCar armMsg;
            memcpy(&armMsg, incomingData, sizeof(armMsg));
            
            // Print to Serial Monitor
            Serial.printf("Received Arm Telemetry -> J1: %d | J2: %d | J3: %d | J4: %d | Gripper: %d\n", 
                           armMsg.joint1, armMsg.joint2, armMsg.joint3, armMsg.joint4, armMsg.gripper);
            
            // Safely write directly to the GUI struct (Removed the I2C mutex block!)
            LiveValues.joint1_angle = armMsg.joint1;
            LiveValues.joint2_angle = armMsg.joint2;
            LiveValues.joint3_angle = armMsg.joint3;
            LiveValues.joint4_angle = armMsg.joint4;
            LiveValues.gripper_val  = armMsg.gripper;
        }
    }
}

// --- Main Control Task ---
static void manualControlTask(void *pvParameters) {
    static bool sequence_ran = false; 
    uint32_t last_packet_time = millis();
    bool is_stopped_by_timeout = true; 
    
    // NEW: Variables to track the One-Shot centering state
    static int last_command = -1;
    static bool centering_done = false; 

    while (true) {
        if (newDataReady) {
            newDataReady = false; 
            
            // Ignore ghost packets
            if (incomingMsg.transX == 0 && incomingMsg.transY == 0 && incomingMsg.rotZ == 0 && incomingMsg.command < 4) {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue; 
            }

            last_packet_time = millis(); 
            is_stopped_by_timeout = false;

            // Handle Valid Commands (2=Velocity, 3=Continuous, 4=Center 350, 5=Center 180)
            if (incomingMsg.command >= 2 && incomingMsg.command <= 5) {
                
                if (incomingMsg.command != last_command) {
                    centering_done = false; 
                    last_command = incomingMsg.command;
                }

                is_autonomous_mode = incomingMsg.isAutonomous;
                
                // 🌟 ADD THIS: Update the Vehicle State for the GUI
                if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
                    LiveValues.vehicleState = is_autonomous_mode ? "Autonomous" : "Manual";
                    xSemaphoreGive(i2c_mutex);
                }

                if (is_autonomous_mode) {
                    if (!sequence_ran) {
                        int8_t stop_cmd[4] = {0, 0, 0, 0};
                        if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
                            motorDriver.setSpeed(stop_cmd);
                            xSemaphoreGive(i2c_mutex); 
                        }
                        trigger_auto_sequence = true;
                        sequence_ran = true;
                    }
                } 
                else if (incomingMsg.command == 4 || incomingMsg.command == 5) {
                    /* === ONE-SHOT AUTO-CENTERING LOGIC === */
                    sequence_ran = false;
                    
                    if (!centering_done) {
                        int target_dist = (incomingMsg.command == 4) ? 350 : 180;
                        int error = current_tof_distance_mm - target_dist;
                        double cmd_vx = 0.0;
                        
                        // Only move if ToF reading is valid
                        if (current_tof_distance_mm > 0 && current_tof_distance_mm < 3000) {
                            if (abs(error) > 15) {
                                double Kp = 0.0015; // Proportional Gain
                                cmd_vx = error * Kp;
                                
                                // Clamp speed bounds safely max 0.15 m/s
                                if (cmd_vx > 0.15) cmd_vx = 0.15;
                                if (cmd_vx < -0.15) cmd_vx = -0.15;
                            } else {
                                // WE REACHED THE TARGET!
                                cmd_vx = 0.0;
                                centering_done = true; // Lock it out so it executes only once
                            }
                        }

                        double wheel_rads[4];
                        kinematics.calculateMotorSpeeds(cmd_vx, 0.0, 0.0, wheel_rads);

                        int8_t motor_rpms[4];
                        for (int i = 0; i < 4; i++) {
                            double rpm = wheel_rads[i] * RADS_TO_RPM;
                            if (isnan(rpm) || isinf(rpm)) rpm = 0.0;
                            else if (rpm > 127.0) rpm = 127.0;
                            else if (rpm < -128.0) rpm = -128.0;
                            motor_rpms[i] = (int8_t)rpm;
                        }
                        
                        if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
                            motorDriver.setSpeed(motor_rpms);
                            xSemaphoreGive(i2c_mutex);
                        }
                    } else {
                        // Centering is complete. Keep motors gracefully stopped.
                        int8_t stop_cmd[4] = {0, 0, 0, 0};
                        if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
                            motorDriver.setSpeed(stop_cmd);
                            xSemaphoreGive(i2c_mutex);
                        }
                    }
                }
                else {
                    /* === NORMAL MANUAL DRIVING === */
                    sequence_ran = false; 
                    
                    TargetVelocity cmd_vel = manualDrive.processInput(incomingMsg);

                    if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
                        LiveValues.vx = cmd_vel.vx;
                        LiveValues.vy = cmd_vel.vy;
                        LiveValues.omega = cmd_vel.omega;
                        xSemaphoreGive(i2c_mutex);
                    }
                    double wheel_rads[4];
                    kinematics.calculateMotorSpeeds(cmd_vel.vx, cmd_vel.vy, cmd_vel.omega, wheel_rads);

                    int8_t motor_rpms[4];
                    for (int i = 0; i < 4; i++) {
                        double rpm = wheel_rads[i] * RADS_TO_RPM;
                        if (isnan(rpm) || isinf(rpm)) rpm = 0.0;
                        else if (rpm > 127.0) rpm = 127.0;
                        else if (rpm < -128.0) rpm = -128.0;
                        motor_rpms[i] = (int8_t)rpm;
                    }
                    
                    if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
                        motorDriver.setSpeed(motor_rpms);
                        xSemaphoreGive(i2c_mutex);
                    }
                }
            }
        }
        else {
            // Watchdog Timeout (300ms without packet)
            if (!is_autonomous_mode && !is_stopped_by_timeout && (millis() - last_packet_time > 300)) {
                int8_t stop_cmd[4] = {0, 0, 0, 0};
                if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
                    motorDriver.setSpeed(stop_cmd);
                    xSemaphoreGive(i2c_mutex);
                }
                is_stopped_by_timeout = true;
                Serial.println("SAFETY STOP: Controller Signal Lost!");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// --- ToF Telemetry Task ---
static void tofTelemetryTask(void *pvParameters) {
    while (true) {
        uint16_t distance = 0xFFFF; 
        
        // 1. Read the sensor safely using the existing I2C mutex
        if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
            distance = tofSensor.readDistance(); 
            xSemaphoreGive(i2c_mutex);           
        }

        // 2. Update the shared structure if reading is valid
        if (distance != 0xFFFF) {
            // Use the mutex to update the shared GUI struct
            if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
                current_tof_distance_mm = (int)distance;
                LiveValues.tofDistance = (int)distance;
                xSemaphoreGive(i2c_mutex);
            }

            // 3. Send ESP-NOW Telemetry for the controller
            outgoingTelemetry.tofDistance_mm = (int)distance;
            esp_now_send(controllerAddress, (uint8_t *)&outgoingTelemetry, sizeof(outgoingTelemetry));
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// --- Init Function ---
void manualcontrol_init() {
    //i2c_mutex = xSemaphoreCreateMutex();

    if (!tofSensor.begin()) Serial.println("ToF Init Failed!");
    else Serial.println("ToF Initialized.");

    WiFi.mode(WIFI_AP_STA); 
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE); // Lock to Channel 1

    if (esp_now_init() != ESP_OK) return;
    esp_now_register_recv_cb(OnDataRecv);

    esp_now_peer_info_t peerInfo = {};
    peerInfo.channel = 1;  
    peerInfo.encrypt = false;
    
    memcpy(peerInfo.peer_addr, controllerAddress, 6);
    esp_now_add_peer(&peerInfo);
    
    memcpy(peerInfo.peer_addr, armAddress, 6);
    esp_now_add_peer(&peerInfo);

    xTaskCreatePinnedToCore(manualControlTask, "ManualTask", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(tofTelemetryTask, "ToFTask", 2048, NULL, 1, &tofTaskHandle, 1);
}
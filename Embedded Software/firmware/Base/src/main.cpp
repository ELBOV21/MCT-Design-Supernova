#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include "MPU9250.h"
#include "HiwonderMotorDriver.h"
#include "Kinematics.h"
#include "Navigator.h"
#include "mission_manager.h" 
#include "GUI.h"
extern volatile bool is_autonomous_mode;
extern volatile bool trigger_auto_sequence;
extern TaskHandle_t tofTaskHandle; 
// --- ADD THESE EXTERN VARIABLES ---
extern SemaphoreHandle_t i2c_mutex; // Bring in the mutex from mission_manager
extern GUI_par LiveValues;          // Bring in the GUI struct
// Bring in the communication variables from mission_manager
extern volatile int arm_ack_state;
extern void send_arm_command(int cmd);

bool RUN_MAG_CALIBRATION = false; 

#define I2C_SDA 21
#define I2C_SCL 22
#define MPU_ADDR 0x68

Preferences prefs;
MPU9250 mpu; 
HiwonderMotorDriver motors(Wire, 0x34);
Kinematics kinematics;
Navigator* navigator;
// --- ADD THE TELEMETRY TASK ---
void telemetryTask(void *pvParameters) {
    while (true) {
        if (!is_autonomous_mode && navigator != nullptr) {
            
            // 1. Update IMU for Global Heading
            navigator->updateIMUYaw();

            // 🌟 2. Update Global Map Coordinates & Encoders
            navigator->updateOdometry();
            
        }
        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
}
void setup() {
    Serial.begin(115200);
    i2c_mutex = xSemaphoreCreateMutex();
    Wire.begin(I2C_SDA, I2C_SCL);
    delay(200);
    
    if (!mpu.setup(MPU_ADDR)) {
        Serial.println("MPU9250 connection failed! Check I2C wiring.");
        while (1) { delay(10); }
    }

    if (!motors.begin(HiwonderMotorDriver::JGB37_520, 0)) {
        Serial.println("Motor driver fail!");
        while (1) { delay(10); }
    }
    int8_t stop_cmd[4] = {0, 0, 0, 0};
    motors.setSpeed(stop_cmd);

     navigator = new Navigator(&motors, &kinematics, &mpu);
    navigator->calibrateGyro();
    Serial.println("Starting Communications...");
    manualcontrol_init(); 
    
    initGUI("ESP32_Robot_Arm", "12345678");
    Serial.println("\n--- All Systems Go! Waiting for Controller ---");

    xTaskCreatePinnedToCore(
        guiTask,   /* Task function */
        "GUITask", /* Name */
        4096,      /* Stack size */
        NULL,      /* Parameters */
        1,         /* Priority */
        NULL,      /* Task handle */
        0          /* Pin to Core 0 */
    );
    xTaskCreatePinnedToCore(
        telemetryTask,   /* Task function */
        "Telemetry",     /* Name */
        4096,            /* Stack size */
        NULL,            /* Parameters */
        1,               /* Priority */
        NULL,            /* Task handle */
        1                /* Pin to Core 1 (Core 0 handles WiFi/GUI) */
    );
   
}


void loop() {
    if (trigger_auto_sequence) {
        trigger_auto_sequence = false; 
        arm_ack_state = 0; // Reset the state before we begin
        
        Serial.println("\n==================================");
        Serial.println("   AUTONOMOUS SEQUENCE STARTED    ");
        Serial.println("==================================");

        if (tofTaskHandle != NULL) {
            vTaskSuspend(tofTaskHandle);
        }

         // --- PHASE 1: RED ---
        navigator->rotate_angle(1.6);
        Serial.println("Auto: Moving Forward");
        //navigator->move_distance(1.0, 0.0); 
        //navigator->move_distance(0.0, 1.5); 
        delay(1000);
        navigator->rotate_angle(-1.6);
        delay(1000);
        navigator->rotate_angle(3.2);
        
        Serial.println("Auto: Commanding Arm to Drop Red...");
        send_arm_command(1); // 1 = Drop Red
        while(arm_ack_state != 2) { delay(10); } // Block until arm replies with 2
        Serial.println("Auto: Red Drop Confirmed!");
        
        // --- PHASE 2: GREEN ---
        Serial.println("Auto: Moving Pure Left");
        navigator->move_distance(0.0, 1.0); 
        
        Serial.println("Auto: Commanding Arm to Drop Green...");
        send_arm_command(3); // 3 = Drop Green
        while(arm_ack_state != 4) { delay(10); } // Block until arm replies with 4
        Serial.println("Auto: Green Drop Confirmed!");
        
        // --- PHASE 3: BLUE ---
        Serial.println("Auto: Moving Diagonal");
        navigator->move_distance(-0.7, -0.76); 
        
        Serial.println("Auto: Commanding Arm to Drop Blue...");
        send_arm_command(5); // 5 = Drop Blue
        while(arm_ack_state != 6) { delay(10); } // Block until arm replies with 6
        Serial.println("Auto: Blue Drop Confirmed!");

        if (tofTaskHandle != NULL) {
            vTaskResume(tofTaskHandle);
        }

        Serial.println("==================================");
        Serial.println("   AUTONOMOUS SEQUENCE COMPLETE   ");
        Serial.println(" Waiting for switch back to Manual");
        Serial.println("==================================");
        
        }
}
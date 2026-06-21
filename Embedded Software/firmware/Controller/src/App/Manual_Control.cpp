#include "Manual_Control.h"
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> 
#include "Joystick.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- Forward Declarations ---
void vCarControlTask(void *pvParameters);

// --- Global Pins ---
#define MODE_SWITCH_PIN 27 // Pushbutton to switch between Car and Arm
#define AUTO_SWITCH_PIN 25 // Pushbutton to toggle Autonomous Mode

// --- Menu Buttons ---
#define BUTTON_UP_PIN 23
#define BUTTON_DOWN_PIN 16
#define BUTTON_SELECT_PIN 15

// --- OLED Configuration ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- Menu Variables ---
const char *menuItems[] = {
    "1. Home", "2. Place Red", "3. Place Green",
    "4. Place Blue", "5. Capture", "6. Grip Position", "7. Pick Position"
};
const int numMenuItems = 7;
int currentMenuIndex = 0;

// Button state tracking
static bool lastUpState = HIGH;
static bool lastDownState = HIGH;
static bool lastSelectState = HIGH;

// Debounce timers
unsigned long lastModeSwitchTime = 0;
unsigned long lastAutoSwitchTime = 0;
const unsigned long debounceDelay = 250; 

// --- Local Objects ---
static Joystick joystickL(32, 33, 13);
static Joystick joystickR(34, 35, 14);

// --- MAC Addresses ---
static uint8_t armAddress[] = {0x80, 0xF3, 0xDA, 0x54, 0x29, 0x1C};
static uint8_t carAddress[] = {0x80, 0xF3, 0xDA, 0x54, 0xAC, 0x58};

// --- Structs ---
typedef struct CarToController {
    int tofDistance_mm;
} CarToController;

struct ControllerToArm {
    int joint1; int joint2; int joint3; int joint4;
    bool Grip; int armPositionCommand; 
    bool ArmSpeed; bool isAutonomous;
};

typedef struct ArmToController {
    int joint0; int joint1; int joint2; int joint3;
} ArmToController;

typedef struct ESPNOW_Message {
    int command;
    int transX; int transY; int rotZ;
    bool isAutonomous;
} ESPNOW_Message;

// --- Callbacks & State Variables ---
static CarToController incomingTelemetry;
static ControllerToArm armData;
static ESPNOW_Message carData;
static ArmToController arm_data;

static bool controlTargetIsArm = false;
static bool lastTargetSwitchState = HIGH;

static bool autonomousMode = false;
static bool lastAutoSwitchState = HIGH;

static bool carMode = false;
static bool prevarmspeed = 0;
static bool gripcommand = 0;

// Base Auto-Centering Tracking: 0 = Manual, 4 = Target 350mm, 5 = Target 200mm
static int baseAutoState = 0; 
bool isDataReady = false;

// --- Consolidated Callback ---
void OnDataReceive(const uint8_t *mac, const uint8_t *incomingData, int len) {
    if (len == sizeof(CarToController)) {
        memcpy(&incomingTelemetry, incomingData, sizeof(incomingTelemetry));
    }
    else if (len == sizeof(ArmToController)) {
        memcpy(&arm_data, incomingData, sizeof(arm_data));
        isDataReady = true;
    }
}

// --- Initialization Function ---
void Car_Control_Init() {
    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE); // Lock Wi-Fi to Channel 1

    esp_now_init();
    esp_now_register_recv_cb(OnDataReceive);

    joystickR.begin();
    joystickL.begin();

    pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);
    pinMode(AUTO_SWITCH_PIN, INPUT_PULLUP);
    pinMode(BUTTON_UP_PIN, INPUT_PULLUP);
    pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);
    pinMode(BUTTON_SELECT_PIN, INPUT_PULLUP);

    Wire.setClock(400000); 
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("SSD1306 allocation failed"));
    }
    display.clearDisplay();
    display.display();

    esp_now_peer_info_t peerInfo = {};
    peerInfo.channel = 1; 
    peerInfo.encrypt = false;

    memcpy(peerInfo.peer_addr, armAddress, 6);
    esp_now_add_peer(&peerInfo);

    memcpy(peerInfo.peer_addr, carAddress, 6);
    esp_now_add_peer(&peerInfo);

    Serial.println("Controller Initialized.");
    xTaskCreate(vCarControlTask, "CarTask", 4096, NULL, 1, NULL);
}

// --- RTOS Task Definition ---
void vCarControlTask(void *pvParameters) {
    for (;;) {
        // --- 1. HANDLE TARGET SWITCHING ---
        bool currentTargetSwitchState = digitalRead(MODE_SWITCH_PIN);
        if (lastTargetSwitchState == HIGH && currentTargetSwitchState == LOW) {
            if (millis() - lastModeSwitchTime > debounceDelay) {
                controlTargetIsArm = !controlTargetIsArm;
                lastModeSwitchTime = millis();
            }
        }
        lastTargetSwitchState = currentTargetSwitchState;

        // --- 2. HANDLE AUTONOMOUS TOGGLE ---
        bool currentAutoSwitchState = digitalRead(AUTO_SWITCH_PIN);
        if (lastAutoSwitchState == HIGH && currentAutoSwitchState == LOW) {
            if (millis() - lastAutoSwitchTime > debounceDelay) {
                autonomousMode = !autonomousMode;
                lastAutoSwitchTime = millis();
            }
        }
        lastAutoSwitchState = currentAutoSwitchState;

        // --- 3. OLED UI RENDERING ---
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);

        if (autonomousMode) {
            display.println("--- AUTONOMOUS ---");
            display.println("System handling ops.");
        }
        else if (!controlTargetIsArm) {
            // === CAR MODE UI ===
            display.println("--- CAR MODE ---");
            display.print("Drive: ");
            display.println(carMode ? "Continuous" : "Velocity");
            
            display.setCursor(0, 30); 
            display.print("ToF Dist: ");
            if (incomingTelemetry.tofDistance_mm > 2000 || incomingTelemetry.tofDistance_mm <= 0) {
                display.println("Out of Range"); 
            } else {
                display.print(incomingTelemetry.tofDistance_mm);
                display.println(" mm");
            }
            
            // Show status if the Base is auto-centering from a previous arm command
            display.setCursor(0, 50);
            if (baseAutoState == 4) display.println("Status: Auto (350mm)");
            else if (baseAutoState == 5) display.println("Status: Auto (200mm)");
            else display.println("Status: Manual");
        }
        else {
            // === ARM MODE UI ===
            display.println("--- ARM POSITIONS ---");
            for (int i = 0; i < numMenuItems; i++) {
                if (i == currentMenuIndex) {
                    display.print(">");
                    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
                } else {
                    display.print(" ");
                    display.setTextColor(SSD1306_WHITE);
                }
                display.println(menuItems[i]);
            }
            display.setTextColor(SSD1306_WHITE);
        }
        display.display();

        // --- 4. READ MENU BUTTONS ---
        bool currentUpState = digitalRead(BUTTON_UP_PIN);
        bool currentDownState = digitalRead(BUTTON_DOWN_PIN);
        bool currentSelectState = digitalRead(BUTTON_SELECT_PIN);

        // --- 5. ROUTE COMMANDS BASED ON TARGET ---
        if (controlTargetIsArm) {
            /* === ARM CONTROL LOGIC === */
            if (joystickL.wasPressed()) { prevarmspeed = !prevarmspeed; }
            if (joystickR.wasPressed()) { gripcommand = !gripcommand; }

            // Menu Scroll UP
            if (lastUpState == HIGH && currentUpState == LOW) {
                currentMenuIndex--;
                if (currentMenuIndex < 0) currentMenuIndex = numMenuItems - 1;
            }
            // Menu Scroll DOWN
            if (lastDownState == HIGH && currentDownState == LOW) {
                currentMenuIndex++;
                if (currentMenuIndex >= numMenuItems) currentMenuIndex = 0;
            }

            // Button Press Logic
            if (currentSelectState == LOW && !autonomousMode) {
                armData.armPositionCommand = currentMenuIndex + 1; 
                
                // Latch the Base Auto State on the initial press
                if (lastSelectState == HIGH) {
                    if (currentMenuIndex == 4) baseAutoState = 4; // 5. Capture = 350mm
                    else if (currentMenuIndex == 6) baseAutoState = 5; // 7. Pick Position = 200mm
                }
            } else {
                armData.armPositionCommand = 0; 
            }

            armData.joint1 = joystickR.getRawX();
            armData.joint2 = joystickR.getRawY();
            armData.joint3 = joystickL.getRawX();
            armData.joint4 = joystickL.getRawY();
            armData.Grip = gripcommand;
            armData.ArmSpeed = prevarmspeed;
            armData.isAutonomous = autonomousMode;

            esp_now_send(armAddress, (uint8_t *)&armData, sizeof(armData));

            // CRUCIAL: Even though we are in Arm Mode, send centering commands to the Base
            if (baseAutoState > 0 && !autonomousMode) {
                carData.command = baseAutoState;
                carData.transX = 0; carData.transY = 0; carData.rotZ = 0;
                carData.isAutonomous = autonomousMode;
                esp_now_send(carAddress, (uint8_t *)&carData, sizeof(carData));
            }

        }
        else {
            /* === CAR CONTROL LOGIC === */
            if (joystickL.wasPressed()) { carMode = !carMode; }

            int jLX = joystickL.getRawX();
            int jLY = joystickL.getRawY();
            int jRX = joystickR.getRawX();

            // Override and cancel auto-centering if the user manually pushes a joystick
            if (abs(jLX - 1850) > 300 || abs(jLY - 1850) > 300 || abs(jRX - 1850) > 300) {
                baseAutoState = 0; // Break out of centering
            }

            if (baseAutoState > 0 && !autonomousMode) {
                // Send Auto-Center Command
                carData.command = baseAutoState;
                carData.transX = 0;
                carData.transY = 0;
                carData.rotZ = 0;
            } else {
                // Send Manual Drive Command
                carData.command = (carMode == false) ? 2 : 3;
                carData.transX = jLX;
                carData.transY = jLY;
                carData.rotZ = jRX;
            }
            
            carData.isAutonomous = autonomousMode;
            esp_now_send(carAddress, (uint8_t *)&carData, sizeof(carData));
        }

        // --- UPDATE BUTTON STATES ---
        lastUpState = currentUpState;
        lastDownState = currentDownState;
        lastSelectState = currentSelectState;

        vTaskDelay(pdMS_TO_TICKS(40)); 
    }
}
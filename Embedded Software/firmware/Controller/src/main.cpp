#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include "Manual_Control.h"
#include "Camera_Control.h"

uint8_t armAddress[] = {0x80, 0xF3, 0xDA, 0x54, 0x29, 0x1C};
/**
 * @brief Main setup - initializes core transport and module tasks
 */

void setup()
{
    Serial.begin(115200);

    // Initialize shared ESP-NOW transport layer
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_send_cb([](const uint8_t *mac_addr, esp_now_send_status_t status)
                             {
                                 // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
                             });

    // Initialize independent control modules
    // Arm_Control_Init();
    Car_Control_Init();
    initCameraComms();
}

/**
 * @brief Default loop deleted to reclaim memory for FreeRTOS
 */
void loop()
{
    vTaskDelete(NULL);
}
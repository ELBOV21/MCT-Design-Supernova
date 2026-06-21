#include "Camera_Control.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// Internal Constants
#define ESPNOW_CHANNEL 1
static uint8_t CAMERA_MAC_ADDR[] = {0x94, 0xA9, 0x90, 0x0C, 0xAE, 0x88};

typedef struct
{
  char payload[128];
} espnow_msg_t;

// State Variables
static volatile bool awaitingAck = false;
static volatile bool systemActive = false;

// --- ESP-NOW Callback ---
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  if (status == ESP_NOW_SEND_SUCCESS && awaitingAck)
  {
    awaitingAck = false;
  }
}

// --- The RTOS Task ---
void cameraCommsTask(void *pvParameters)
{
  // Setup Input Pin
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);

  // Setup Radio
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK)
  {
    vTaskDelete(NULL);
    return;
  }

  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t cameraPeer = {};
  memcpy(cameraPeer.peer_addr, CAMERA_MAC_ADDR, 6);
  cameraPeer.channel = ESPNOW_CHANNEL;
  cameraPeer.encrypt = false;
  esp_now_add_peer(&cameraPeer);

  int lastState = HIGH;

  for (;;)
  {
    int reading = digitalRead(START_BUTTON_PIN);

    // Press Detection (Active Low)
    if (reading == LOW && lastState == HIGH && !systemActive)
    {
      systemActive = true;
      awaitingAck = true;

      espnow_msg_t msg;
      memset(msg.payload, 0, sizeof(msg.payload));
      strcpy(msg.payload, "START");

      // Background Retry Loop
      while (awaitingAck)
      {
        esp_now_send(CAMERA_MAC_ADDR, (uint8_t *)&msg, sizeof(msg));
        Serial.println("Sent START command, awaiting ACK...");
        // Block this task for 200ms to allow radio to process
        vTaskDelay(pdMS_TO_TICKS(200));
      }

      systemActive = false;
    }

    lastState = reading;
    // Check button every 50ms
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void initCameraComms()
{
  // Priority set to 1. Increase if this needs to be more urgent than main logic.
  xTaskCreate(cameraCommsTask, "CamComms", 4096, NULL, 1, NULL);
}
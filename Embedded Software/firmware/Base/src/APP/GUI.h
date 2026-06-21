#ifndef GUI_H
#define GUI_H

#include <WiFi.h>
#include <WebServer.h>

struct GUI_par {
    float roll;
    float yaw;
    float pitch;
    float encoder1;
    float encoder2;
    float encoder3;
    float encoder4;
    
    float vx;
    float vy;
    float omega; 
    
    // 🌟 ADD THE COORDINATES HERE
    float x;
    float y;
    
    String vehicleState; 
    
    int tofDistance;
    float joint1_angle;
    float joint2_angle;
    float joint3_angle;
    float joint4_angle;
    float gripper_val;
};

void initGUI(const char* ssid, const char* password);
void handleDataRequest();
void updateGUI(GUI_par &data);
void displayReadings(const GUI_par &data);
void guiTask(void *pvParameters);

#endif
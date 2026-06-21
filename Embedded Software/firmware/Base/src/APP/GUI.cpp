#include "GUI.h"

WebServer guiServer(80);
GUI_par LiveValues; 

void initGUI(const char* ssid, const char* password) {
    WiFi.softAP(ssid, password);
    
    Serial.println("\nAP Started!");
    Serial.print("ESP32 IP Address: ");
    Serial.println(WiFi.softAPIP()); // This will print 192.168.4.1
    // -------------------------
    
    guiServer.on("/data", handleDataRequest);
    guiServer.begin();
    
}

// --- Replace this function in gui.cpp ---
void handleDataRequest() {
    String json = "{";
    json += "\"yaw\":" + String(LiveValues.yaw) + ",";
    json += "\"pitch\":" + String(LiveValues.pitch) + ",";
    json += "\"roll\":" + String(LiveValues.roll) + ",";
    json += "\"encoder1\":" + String(LiveValues.encoder1) + ",";
    json += "\"encoder2\":" + String(LiveValues.encoder2) + ",";
    json += "\"encoder3\":" + String(LiveValues.encoder3) + ",";
    json += "\"encoder4\":" + String(LiveValues.encoder4) + ",";
    json += "\"vx\":" + String(LiveValues.vx) + ",";
    json += "\"vy\":" + String(LiveValues.vy) + ",";
    json += "\"omega\":" + String(LiveValues.omega) + ",";
    json += "\"x\":" + String(LiveValues.x) + ",";
    json += "\"y\":" + String(LiveValues.y) + ",";
    json += "\"vehiclestate\":\"" + LiveValues.vehicleState + "\",";
    json += "\"tof\":" + String(LiveValues.tofDistance) + ",";
    
    // Append the arm values into the server response payload
    json += "\"j1\":" + String(LiveValues.joint1_angle) + ",";
    json += "\"j2\":" + String(LiveValues.joint2_angle) + ",";
    json += "\"j3\":" + String(LiveValues.joint3_angle) + ",";
    json += "\"j4\":" + String(LiveValues.joint4_angle) + ",";
    json += "\"g\":"  + String(LiveValues.gripper_val);
    json += "}";
    
    guiServer.send(200, "application/json", json);
}

void updateGUI(GUI_par &newData) {
    LiveValues = newData;
    guiServer.handleClient();
}

void displayReadings(const GUI_par &data) {
    Serial.println("\n--- TELEMETRY SYSTEM WORKSPACE ---");
    Serial.printf("ARM JOINTS -> J1:%.1f | J2:%.1f | J3:%.1f | J4:%.1f\n", 
                  data.joint1_angle, data.joint2_angle, data.joint3_angle, data.joint4_angle);
    Serial.printf("GLOBAL COORDINATES -> X:%.1f | Y:%.1f\n", data.x, data.y);
    Serial.println("---------------------------------");
}

// --- Add this to GUI.cpp ---
void guiTask(void *pvParameters) {
    for (;;) {
        // Handle incoming web requests
        guiServer.handleClient();
        
        // Small delay to yield to other tasks
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}
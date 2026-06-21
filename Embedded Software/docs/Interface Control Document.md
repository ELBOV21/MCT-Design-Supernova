&#x20;System Interface Control Document (ICD)



8.1 Network Topology \& Physical Layer



The robotic ecosystem utilizes a hybrid network architecture operating over the 2.4 GHz spectrum to balance low-latency control with high-bandwidth telemetry.



Low-Latency Control (ESP-NOW): The Controller, Base, Arm, and Camera utilize the ESP-NOW protocol locked to Wi-Fi Channel 1. This peer-to-peer MAC-layer protocol bypasses standard Wi-Fi overhead, ensuring deterministic, low-latency transmission (< 5ms) critical for real-time motor control and synchronization.



High-Bandwidth Streaming (Wi-Fi 802.11): The Camera and Graphical User Interface (GUI) utilize a standard Wi-Fi network to support high-bandwidth video streaming and color detection overlays.



Table 8.1: Network Node Registry



Node Identifier



Hardware Platform



Network Type



Primary Role



Controller



ESP32 (HMI)



ESP-NOW



Master input router. Broadcasts kinematics and triggers.



Mobile Base



ESP32 (Core)



ESP-NOW



Executes driving kinematics. Relays system telemetry to GUI.



Manipulator Arm



ESP32 (Peripheral)



ESP-NOW



Executes pick/place operations. Broadcasts joint telemetry.



Vision Camera



ESP32-CAM



ESP-NOW \& Wi-Fi



Awaits START trigger. Streams video/color data to GUI.



GUI Dashboard



PC / Web Client



Wi-Fi



Renders real-time digital twin, odometry, and camera stream.



8.2 Data Payloads \& Structures



Communication across the ESP-NOW network is executed via tightly packed C-structs to minimize airtime and reduce packet collisions.



8.2.1 Controller → Mobile Base (Kinematics Command)



Streams continuously at 25Hz during manual operation. Translates physical joystick vectors into mobile base driving instructions.



typedef struct ESPNOW\_Message {

&#x20;   int command;       // State selector (2=Snapped, 3=Continuous, 4=Auto350, 5=Auto180)

&#x20;   int transX;        // Raw X-axis joystick input (0-4095)

&#x20;   int transY;        // Raw Y-axis joystick input (0-4095)

&#x20;   int rotZ;          // Raw Z-axis rotational input (0-4095)

&#x20;   bool isAutonomous; // Boolean trigger for the main autonomous sequence

} ESPNOW\_Message;





8.2.2 Controller → Manipulator Arm (Pose Command)



Streams continuously at 25Hz/10Hz. Bypasses the base to control the end-effector directly.



struct ControllerToArm {

&#x20;   int joint1; int joint2; int joint3; int joint4; // Raw Joystick telemetry

&#x20;   bool Grip;               // True = Close Gripper

&#x20;   int armPositionCommand;  // Pre-programmed spatial array index (1-7)

&#x20;   bool ArmSpeed;           // True = Fast Mode (25ms), False = Slow (100ms)

&#x20;   bool isAutonomous;       // Sequence trigger flag

};





8.2.3 Base ↔ Arm (Sequence Synchronization \& Telemetry)



The Base and Arm share an execution synchronization struct (BaseArmSync) to coordinate blocking movements. The Arm also continuously broadcasts its physical state back to the Base at 20Hz.



// Handshake Trigger/ACK

struct BaseArmSync {

&#x20;   int auto\_command; // 1,3,5 = Base commanding Drop. 2,4,6 = Arm Acknowledging.

};



// Continuous Telemetry Feedback

struct ArmTelemetryToCar {

&#x20;   int joint1; int joint2; int joint3; int joint4; // Current physical angles

&#x20;   int gripper;                                    // Current gripper angle

};





8.2.4 Mobile Base → GUI Dashboard (Digital Twin Telemetry)



The Base aggregates fused sensor data and streams this unified payload to the PC GUI.



struct GUI\_par {

&#x20;   // IMU Data

&#x20;   float roll; float yaw; float pitch;

&#x20;   

&#x20;   // Motor Odometry

&#x20;   float encoder1; float encoder2; float encoder3; float encoder4;

&#x20;   float vx; float vy; float omega; 

&#x20;   float x; float y; // Global integrated position

&#x20;   

&#x20;   // Status

&#x20;   char vehicleState\[32]; // Note: Fixed array used for network safety

&#x20;   int tofDistance;

&#x20;   

&#x20;   // Arm Telemetry

&#x20;   float joint1\_angle; float joint2\_angle; 

&#x20;   float joint3\_angle; float joint4\_angle;

&#x20;   float gripper\_val;

};





8.3 Handshaking Protocols and Timing



8.3.1 Guaranteed Delivery (Controller → Camera)



To ensure reliable activation in noisy RF environments, the Controller implements a Guaranteed Delivery loop for the vision trigger.



The operator presses the "START" button.



The Controller broadcasts an espnow\_msg\_t containing the string "START".



The FreeRTOS task blocks and enters a while(awaitingAck) retry loop.



The transmission repeats until the ESP32 hardware fires the onDataSent interrupt confirming a physical-layer MAC acknowledgment.



8.3.2 Blocking Sequence Synchronization (Base ↔ Arm)



During the AUTO\_SEQUENCE phase, the Base Module halts its navigation thread to allow the Arm to complete physical payload placement.



Base arrives at Waypoint.



Base transmits BaseArmSync with auto\_command = {1, 3, or 5}.



Base executes a blocking while(arm\_ack\_state != {2, 4, or 6}) loop, suspending navigation.



Arm executes the quintic trajectory sequence.



Arm transmits BaseArmSync with corresponding ACK.



Base breaks the while-loop and resumes navigation.



8.3.3 Failsafe Watchdog Timer



During all manual teleoperation modes, the system mandates a continuous heartbeat. If the timestamp delta between received packets exceeds 300ms, the system logs a communication loss and automatically halts all PWM signals to motor drivers to prevent runaway behavior.


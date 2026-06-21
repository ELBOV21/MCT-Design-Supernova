# VDI 2206: Distributed Mechatronic Mobile Manipulator

![C++](https://img.shields.io/badge/Language-C++-blue)
![PlatformIO](https://img.shields.io/badge/Build_System-PlatformIO-orange)
![RTOS](https://img.shields.io/badge/OS-FreeRTOS-lightgrey)
![Architecture](https://img.shields.io/badge/Network-ESP--NOW%20%7C%20Wi--Fi-success)

This repository contains the complete firmware, software, and systems engineering documentation for a distributed, multi-node holonomic mobile manipulator. Developed according to the **VDI 2206** mechatronic design methodology, the system executes complex kinematic trajectories, autonomous QR-code waypoint navigation, and high-frequency sensor fusion across five isolated computational nodes.

---

## 🏗️ System Architecture

The ecosystem relies on a decentralized microservices hardware architecture. Real-time control and kinematics run on bare-metal or FreeRTOS, communicating over a deterministic MAC-layer protocol, while heavy telemetry is offloaded to standard Wi-Fi.

### Hardware Nodes
1. **HMI Controller (ESP32):** The master handheld controller. Multiplexes dual analog joysticks and I2C OLED UI rendering over a 25Hz FreeRTOS task.
2. **Mobile Base (ESP32):** The central execution hub. Computes holonomic inverse kinematics, integrates MPU9250 IMU and VL53L0X ToF sensor data, and executes blocking autonomous drop-off sequences.
3. **Manipulator Arm (ESP32):** A deterministic peripheral node. Executes 5th-order quintic polynomials via a 5-tier Priority Queue to drive a PCA9685 PWM expansion board.
4. **Vision Camera (ESP32-S3-EYE):** An isolated optical node. Runs a dual-core architecture: Core 1 handles continuous MJPEG Wi-Fi streaming, while Core 0 runs on-demand `quirc` C-library QR decoding.
5. **GUI Dashboard (PC):** A digital twin that parses UDP/IP telemetry packets to visualize odometry, ToF distances, and live arm joint angles.

### Network Topology
* **Low-Latency Control (ESP-NOW):** Time-critical kinematic commands and bidirectional handshake sequences are transmitted via peer-to-peer ESP-NOW locked to Wi-Fi Channel 1 for sub-5ms latency.
* **Telemetry & Vision (Wi-Fi UDP/HTTP):** Sensor fusion structs (`GUI_par`) and video streams are transmitted over standard 802.11 Wi-Fi.
* *Note on Vision Node:* The ESP32-S3 utilizes a `WIFI_AP_STA` "Channel Lock" workaround, hosting a hidden SoftAP to force the hardware radio onto Channel 1, preventing ESP-NOW packet loss while connected to the router.

---

## 📁 Repository Structure (Monorepo)

To ensure Interface Control Document (ICD) compliance across all nodes, this project is structured as a monorepo. Shared network payloads are centralized to prevent struct misalignment.

```text
├── docs/                       # VDI 2206 Engineering Documentation
│   ├── System_Architecture.md
│   ├── Interface_Control_Document.md
│   └── Vision_Camera_Specs.md
│
├── shared/                     # Centralized ICD Data Structures
│   └── network_payloads.h      # Shared ESP-NOW C-structs 
│
├── firmware/                   # Microcontroller Source Code
│   ├── HMI_Controller/         # UI routing and joystick debouncing
│   ├── Mobile_Base/            # Kinematics, ToF centering, and Nav sequences
│   ├── Manipulator_Arm/        # Quintic trajectory planning and PWM driving
│   └── Vision_Camera/          # QR Detection (quirc) and HTTP stream
│
└── software/                   # PC-Side Applications
    └── GUI_Dashboard/          # Python/JS Digital Twin interface

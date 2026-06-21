# VDI 2206: Distributed Mechatronic Mobile Manipulator
This repository contains the complete systems engineering files for a distributed, multi-node holonomic mobile manipulator. Developed according to the **VDI 2206** mechatronic design methodology, the system executes complex kinematic trajectories, autonomous QR-code waypoint navigation, and high-frequency sensor fusion. 

This monorepo unifies the three core engineering domains—**Software, Electrical, and Modeling & Simulation**—ensuring cross-disciplinary synchronization across the entire project lifecycle.

---

## 📁 Repository Structure

The project is divided into three primary domains to keep the workspace organized:

```text
├── software/                   # Microcontroller Firmware & PC Applications
│   ├── shared/                 # Centralized ICD (Interface Control Document) structs
│   ├── firmware/               # PlatformIO ESP32 projects (Base, Arm, HMI, Camera)
│   └── GUI_Dashboard/          # Remote digital twin and telemetry viewer
│
├── electrical/                 # Schematics, PCBs, and Power Distribution
│   ├── schematics/             # Wiring diagrams for ESP32 breakout & internal busses
│   ├── pcb_layouts/            # Custom PCB design files and Gerber exports
│   └── datasheets/             # Component specs (Hiwonder driver, MPU9250, VL53L0X, PCA9685)
│
├── modeling_and_simulation/    # Kinematics, Dynamics, and Control Theory
│   ├── MATLAB/                 # Forward/Inverse kinematics, Jacobian analysis, PID tuning
│   └── Simulink/               # Dynamic modeling of motor responses and holonomic base
│
└── README.md                   # Master project documentation

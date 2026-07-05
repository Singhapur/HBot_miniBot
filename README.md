# 🤖 HBot_miniBot

Final Degree Project (Bachelor’s Thesis) repository to build a **miniBot** based on a distributed architecture using Arduino, Raspberry Pi, and ESP32.

This project documents the complete process: from physical construction (CAD design and 3D printing) to programming and integration of the different systems using ROS 2 and AI computer vision.
---

# 📌 Description

**HBot_miniBot** is an advanced educational and practical project that combines:

- **Low-Level Hardware Control:** Arduino for strict real-time motor PWM control and ESP32 for high-frequency (200Hz) telemetry (IMU & LiDAR).
- **High-Level Processing:** Raspberry Pi 5 acting as the brain, running ROS 2 (Ubuntu Server) for odometry, waypoint navigation, and Visual Servoing.
- **Artificial Intelligence:** Real-time human tracking using the YOLO11-pose model (Ultralytics) running directly on the edge.
- **Wireless Communication:** DDS network topology allowing seamless monitoring and teleoperation from a remote laptop via RViz2.

---

# 🏗 Construction

This section describes the complete miniBot building process. The chassis is 100% custom-designed focusing on modularity and maintenance.

- **Chassis design:** Modeled in CAD and optimized for FDM 3D printing.
- **Prototyping:** Dimensional validation performed using wood cardboard before final manufacturing.
- **Fabrication:** Final structural parts printed in PETG using an Original Prusa MK3S for superior thermal and mechanical resistance.
- **Hardware integration:** Tiered design (drive level, logic level, and sensor level). 

Images, schematics, and assembly recommendations will be included here.
<img width="1141" height="782" alt="imagen" src="https://github.com/user-attachments/assets/4212aafd-a856-49ff-af52-a1f346216cc1" />


---

# 📦 Bill of Materials (BOM)

## 🔩 Electronics

- Raspberry Pi 5 (Main brain)
- Arduino Uno / Nano (Motor & Actuator controller)
- ESP32 (Sensor bridge)
- Motor driver (L298N, L293D Shield, or similar)
- 1D LiDAR / Distance Sensor
- IMU (Inertial Measurement Unit)
- USB Camera
- 7.4V LiPo Battery (for motors)
- External power bank (5V 3A) for Raspberry Pi
- Breadboard and Dupont wires

## ⚙ Mechanics

- Custom 3D-Printed chassis (PETG)
- 4x TT DC motors with wheels and encoders
- 1x Servomotor (for camera/sensor panning)
- M3 Screws and nuts
- Brass standoffs / Spacers

---

# 💻 Software

The software architecture is divided into microcontroller firmware and a ROS 2 workspace. 

## 1. Microcontroller Firmware (C/C++)
Located in the `/arduino_esp_code` folder:
- **Arduino Code (arduino_code.ino):** Handles hardware interrupts for wheel encoders and translates binary serial commands into PWM signals for the DC motors. Includes an emergency stop trigger.
- **ESP32 Code (MPU_Luna.ino):** Samples the IMU and distance sensors at high speeds and streams the data to the Pi using a custom binary protocol with Checksum validation.

## 2. ROS 2 Workspace (Python)
Located in the `/robot_tfg_pkg/robot_tfg_pkg` folder. This is the core logic of the robot:

- `arduino_bridge.py` & `esp32_bridge.py`: Serial interfaces translating raw binary data to standard ROS 2 messages (`Twist`, `Imu`).
- `odometry_node.py`: Fuses encoder data with IMU gyroscope readings to estimate the robot's real-time position in the world.
- `camera_reader_yolo.py`: Captures video, runs YOLO11n-pose inference, and calculates visual servoing errors to center the target using the servo and chassis movement.
- `high_level_controller.py`: The main Finite State Machine (FSM). Handles pure alignment, waypoint navigation, deadband compensation, and emergency braking.
- `waypoint_sender.py`: A node that safely dispatches navigation coordinates once the DDS network is fully discovered.

---

# 🔧 Assembly

General assembly steps:

1. Validate dimensions with cardboard prototypes.
2. 3D Print the chassis layers in PETG.
3. Mount the TT motors and encoders onto the bottom chassis.
4. Attach the motor driver and power distribution system.
5. Install Arduino, Raspberry Pi, and ESP32 on their respective deck levels.
6. Mount the articulated neck for the camera and the top turret for the LiDAR.
7. Verify electrical connections (⚠ **Important:** Check polarities before powering the system).

## Testing Model

In this model we will see how the 3D model would actually look like, and also be able to start programming the functionalities.

- Base without ESP32 and Arduino case
  
  <img src="https://github.com/user-attachments/assets/854e4cd7-0859-48c9-87d3-7aae2e97bb40" width="400">

- Completed base

  <img src="https://github.com/user-attachments/assets/d5888c76-8e36-4dbe-b7a3-ffff88725b03" width="400">

- Testing model completed

  <img src="https://github.com/user-attachments/assets/2646a9ea-567d-455a-a222-1a9dff667bab" width="400">
  
## 3D Printed Model
<img width="400" src="https://github.com/user-attachments/assets/df6cddc4-0fe3-4c2d-a317-46d83a9e4a20" />


# 🤖 HBot_miniBot

Final Degree Project (Bachelor’s Thesis) repository to build a **miniBot** based on Arduino, Raspberry Pi, and ESP32.

This project documents the complete process: from physical construction to programming and integration of the different systems.

---

# 📌 Description

**HBot_miniBot** is an educational and practical project that combines:

- High-level control with Arduino and ESP32  
- Use of ROS2 with Raspberry Pi  
- Wireless communication from laptop to Raspberry Pi  

---

# 🏗 Construction

This section describes the complete miniBot building process:

- Chassis design  
- Motor assembly  
- Sensor installation  
- Power distribution  
- Hardware integration  

Images, schematics, and assembly recommendations will be included here.
<img width="1141" height="782" alt="imagen" src="https://github.com/user-attachments/assets/4212aafd-a856-49ff-af52-a1f346216cc1" />


---

# 📦 Bill of Materials (BOM)

## 🔩 Electronics

- Arduino Uno (or compatible)  
- Raspberry Pi 5  
- ESP32  
- Motor driver (L293D Shield or similar)  
- Sensors, encoders, and distance sensors  
- Battery (7.4V or similar)
- External power bank of 5V and 3A (for power Raspberry Pi)  
- Breadboard and Dupont wires  

## ⚙ Mechanics

- Mini robot chassis  
- 4x TT DC motors with wheels  
- 1x Servo  
- Screws  
- Spacers  

---

# 🔧 Assembly

General assembly steps:

1. Mount the motors onto the chassis  
2. Attach the motor driver  
3. Install Arduino, Raspberry Pi, and ESP32  
4. Connect sensors and the rest of the hardware  
5. Verify electrical connections  
6. Perform power supply tests  

⚠ Important: Check polarities before powering the system.

## Testing model

In this model we will see what the 3D model would actually look like, and also be able to start programming the functionalities.

- Base without ESP32 and Arduino case
  
  <img src="https://github.com/user-attachments/assets/854e4cd7-0859-48c9-87d3-7aae2e97bb40" width="400">

- Camera support

  <img src="https://github.com/user-attachments/assets/d79d7b74-5a7f-410a-8e81-a540ed685b36" width="400">

---

# 💻 Software

Project structure:

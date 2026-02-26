# 🤖 HBot_miniBot

Repositorio del TFG (Trabajo Final de Grado) para construir un **miniBot** basado en Arduino, Raspberry Pi y ESP32.

Este proyecto documenta el proceso completo: desde la construcción física hasta la programación e integración de los distintos sistemas.

---

# 📌 Descripción

**HBot_miniBot** es un proyecto educativo y práctico que combina:

-  Control de alto nivel con Arduino y ESP32
-  Uso de ROS2 con Raspberry Pi  
-  Comunicación inalámbrica del portatil a la Rasp

---

# 🏗 Construction

Esta sección describe el proceso completo de construcción del miniBot:

- Diseño del chasis  
- Montaje de motores  
- Instalación de sensores  
- Distribución de alimentación  
- Integración de el hardware

Aquí se incluirán imágenes, esquemas y recomendaciones de montaje.

---

# 📦 Bill of Materials (BOM)

## 🔩 Electrónica

- Arduino Uno (o compatible)  
- Raspberry Pi 5 
- ESP32  
- Driver de motores (L293D Shield o similar)
- Sensores, encoders y sensores de distancia
- Batería (7.4V o similar)  
- Protoboard y cables Dupont  

## ⚙ Mecánica

- Chasis para mini robot  
- 4x Motores TT DC con ruedas
- 1x Servo
- Tornillería  
- Separadores  

---

# 🔧 Assembly

Pasos generales de ensamblaje:

1. Montar los motores en el chasis  
3. Fijar el driver de motores  
4. Instalar Arduino, Raspberry Pi y ESP32  
5. Conectar sensores y la resta del hardware
6. Verificar conexiones eléctricas  
7. Realizar pruebas de alimentación  

⚠ Importante: comprobar polaridades antes de energizar el sistema.

---

# 💻 Software

Estructura del proyecto:

```

```
---

## 🔹 Arduino Code

Funciones principales:

- Control de motores y de servo 
- Comunicación serial con Raspberry Pi  

### Requisitos:
- Arduino IDE  

---

## 🔹 Raspberry Pi Code

Funciones principales:

- Lógica de control principal  
- Procesamiento de datos  
- Comunicación con Arduino y la ESP32
- Posible integración con visión artificial  

### Requisitos:
- Ubuntu 24.04 or Ubuntu 24.04 Server  
- Python 3.x  

---

## 🔹 ESP32 Code

Funciones principales:


### Requisitos:
- Arduino IDE o PlatformIO  

---

# 🎯 Objetivos del Proyecto

- Integrar múltiples microcontroladores  
- Implementar comunicación serial y/o inalámbrica  
- Crear una base modular para robótica educativa  
- Facilitar futuras ampliaciones  

---

# 📈 Mejoras Futuras

- Control vía app móvil  
- Dashboard web  
- Integración IoT  
- Sistema autónomo con visión  

---

# 👨‍💻 Autor

Proyecto desarrollado por **HBot**.

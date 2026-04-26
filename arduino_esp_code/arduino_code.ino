#include "AFMotor_R4.h"
#include <Servo.h>
#include <EnableInterrupt.h>

// --- PROTOCOLO DE COMUNICACIÓN BINARIA ---
#define START_BYTE 0xAA
#define END_BYTE   0x55
#define ID_ENCODERS 0x02
#define ID_CMD_MOTOR_SERVO 0x10

// --- CONFIGURACIÓN MOTORES DC ---
AF_DCMotor M1(1); // Front left
AF_DCMotor M2(2); // Front right
AF_DCMotor M3(3); // Back right
AF_DCMotor M4(4); // Back left
// --- CONFIGURACIÓN DE LOS 2 SERVOS ---
Servo miServoCamara;
Servo miServoLidar;

// --- ESTADOS DEL ROBOT ---
int pwm_izq = 0; int dir_izq = RELEASE;
int pwm_der = 0; int dir_der = RELEASE;
int angulo_camara = 90;
int angulo_lidar = 90;
int motor_watchdog = 0; // Perro guardián

// --- ENCODERS ---
const byte pinEncFI = A0;
const byte pinEncFD = A1;
const byte pinEncTI = A2;
const byte pinEncTD = A3;

// Usamos uint8_t porque enviamos la diferencia (delta) cada 50ms
volatile uint8_t ticks_FI = 0;
volatile uint8_t ticks_FD = 0;
volatile uint8_t ticks_TI = 0;
volatile uint8_t ticks_TD = 0;

uint8_t buffer_encoders[4];
unsigned long last_loop_time = 0;

// --- INTERRUPCIONES (PCINT) ---
void contarFI() { ticks_FI++; }
void contarFD() { ticks_FD++; }
void contarTI() { ticks_TI++; }
void contarTD() { ticks_TD++; }

// ==========================================
// FUNCIÓN PARA ENVIAR BINARIO
// ==========================================
void sendMessage(uint8_t id, uint8_t* data, uint8_t len) {
  Serial.write(START_BYTE);
  Serial.write(id);
  Serial.write(len);
  uint8_t checksum = id ^ len; 
  for (uint8_t i = 0; i < len; i++) {
    Serial.write(data[i]);
    checksum ^= data[i]; 
  }
  Serial.write(checksum);
  Serial.write(END_BYTE);
}

// ==========================================
// MÁQUINA DE ESTADOS PARA RECIBIR BINARIO
// ==========================================
void receiveMessage() {
  static enum { WAIT_START, READ_ID, READ_LEN, READ_DATA, READ_CHK, WAIT_END } state = WAIT_START;
  static uint8_t buffer[10];
  static uint8_t idx = 0;
  static uint8_t id, len, checksum;

  while (Serial.available()) {
    uint8_t byte_in = Serial.read();
    switch (state) {
      case WAIT_START:
        if (byte_in == START_BYTE) state = READ_ID;
        break;
      case READ_ID:
        id = byte_in; checksum = id; state = READ_LEN;
        break;
      case READ_LEN:
        len = byte_in; checksum ^= len; idx = 0;
        if(len > 10) state = WAIT_START; 
        else state = READ_DATA;
        break;
      case READ_DATA:
        buffer[idx++] = byte_in; checksum ^= byte_in;
        if (idx >= len) state = READ_CHK;
        break;
      case READ_CHK:
        if (byte_in == checksum) state = WAIT_END;
        else state = WAIT_START; 
        break;
      case WAIT_END:
        if (byte_in == END_BYTE) {
          // Si es un comando de movimiento y trae 6 DATOS (MotorI, PwmI, MotorD, PwmD, Camara, Lidar)
          if (id == ID_CMD_MOTOR_SERVO && len == 6) {
            
            dir_izq = (buffer[0] == 0) ? BACKWARD : (buffer[0] == 1 ? FORWARD : RELEASE);
            pwm_izq = buffer[1];
            
            dir_der = (buffer[2] == 0) ? BACKWARD : (buffer[2] == 1 ? FORWARD : RELEASE);
            pwm_der = buffer[3];
            
            angulo_camara = buffer[4];
            angulo_lidar = buffer[5];
            
            motor_watchdog = 10; // Reiniciamos el perro guardián (500ms)
          }
        }
        state = WAIT_START;
        break;
    }
  }
}

// ==========================================
// APLICAR LOS VALORES AL HARDWARE
// ==========================================
void applyHardware() {
  M1.run(dir_izq); M1.setSpeed(pwm_izq);
  M4.run(dir_izq); M4.setSpeed(pwm_izq);
  
  M2.run(dir_der); M2.setSpeed(pwm_der);
  M3.run(dir_der); M3.setSpeed(pwm_der);
  
  
  miServoCamara.write(angulo_camara);
  miServoLidar.write(angulo_lidar);
}

// Código para Arduino
void setup() {
  Serial.begin(115200);
  
  miServoCamara.attach(10);
  miServoCamara.write(angulo_camara);
  
  miServoLidar.attach(9);
  miServoLidar.write(angulo_lidar);

  pinMode(pinEncFI, INPUT_PULLUP); pinMode(pinEncFD, INPUT_PULLUP);
  pinMode(pinEncTI, INPUT_PULLUP); pinMode(pinEncTD, INPUT_PULLUP);

  enableInterrupt(pinEncFI, contarFI, RISING);
  enableInterrupt(pinEncFD, contarFD, RISING);
  enableInterrupt(pinEncTI, contarTI, RISING);
  enableInterrupt(pinEncTD, contarTD, RISING);
}

void loop() {
  unsigned long now = millis();

  // 1. ESCUCHAR A LA RASPBERRY SIN BLOQUEOS
  receiveMessage();

  // 2. EJECUTAR ACCIONES CADA 50ms
  if (now - last_loop_time >= 50) {
    last_loop_time = now;

    // Recoger encoders pausando interrupciones un instante
    noInterrupts();
    buffer_encoders[0] = ticks_FI; ticks_FI = 0;
    buffer_encoders[1] = ticks_FD; ticks_FD = 0;
    buffer_encoders[2] = ticks_TI; ticks_TI = 0;
    buffer_encoders[3] = ticks_TD; ticks_TD = 0;
    interrupts();

    // Enviar odometría
    sendMessage(ID_ENCODERS, buffer_encoders, 4);

    // Watchdog
    if (motor_watchdog > 0) {
      motor_watchdog--;
    } else {
      dir_izq = RELEASE; pwm_izq = 0;
      dir_der = RELEASE; pwm_der = 0;
    }

    // Actualizar motores y servos
    applyHardware();
  }
}

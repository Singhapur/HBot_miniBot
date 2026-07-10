#include "AFMotor_R4.h"
#include <Servo.h>
#include <EnableInterrupt.h>

// --- BINARY COMMUNICATION PROTOCOL ---
#define START_BYTE 0xAA
#define END_BYTE   0x55
#define ID_ENCODERS 0x02
#define ID_CMD_MOTOR_SERVO 0x10

// --- DC MOTOR CONFIGURATION ---
AF_DCMotor M1(1); // Front left
AF_DCMotor M2(2); // Front right
AF_DCMotor M3(3); // Rear right
AF_DCMotor M4(4); // Rear left

// --- TWO SERVO CONFIGURATION ---
Servo cameraServo;
Servo lidarServo;

// --- ROBOT STATES ---
int left_pwm = 0;
int left_dir = RELEASE;
int right_pwm = 0;
int right_dir = RELEASE;
int camera_angle = 90;
int lidar_angle = 90;
int motor_watchdog = 0; // Watchdog timer

// --- ENCODERS ---
const byte pinEncFL = A0;
const byte pinEncFR = A1;
const byte pinEncRL = A2;
const byte pinEncRR = A3;

// We use uint8_t because we send the encoder delta every 50 ms
volatile uint8_t ticks_FL = 0;
volatile uint8_t ticks_FR = 0;
volatile uint8_t ticks_RL = 0;
volatile uint8_t ticks_RR = 0;

uint8_t encoder_buffer[4];
unsigned long last_loop_time = 0;

// --- INTERRUPT ROUTINES (PCINT) ---
void countFL() { ticks_FL++; }
void countFR() { ticks_FR++; }
void countRL() { ticks_RL++; }
void countRR() { ticks_RR++; }

// ==========================================
// FUNCTION TO SEND BINARY DATA
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
// STATE MACHINE TO RECEIVE BINARY DATA
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
        id = byte_in;
        checksum = id;
        state = READ_LEN;
        break;

      case READ_LEN:
        len = byte_in;
        checksum ^= len;
        idx = 0;

        if (len > 10)
          state = WAIT_START;
        else
          state = READ_DATA;
        break;

      case READ_DATA:
        buffer[idx++] = byte_in;
        checksum ^= byte_in;

        if (idx >= len)
          state = READ_CHK;
        break;

      case READ_CHK:
        if (byte_in == checksum)
          state = WAIT_END;
        else
          state = WAIT_START;
        break;

      case WAIT_END:
        if (byte_in == END_BYTE) {
          // If this is a motion command containing 6 data bytes
          // (LeftDir, LeftPWM, RightDir, RightPWM, Camera, LiDAR)
          if (id == ID_CMD_MOTOR_SERVO && len == 6) {

            left_dir = (buffer[0] == 0) ? BACKWARD :
                       (buffer[0] == 1) ? FORWARD : RELEASE;
            left_pwm = buffer[1];

            right_dir = (buffer[2] == 0) ? BACKWARD :
                        (buffer[2] == 1) ? FORWARD : RELEASE;
            right_pwm = buffer[3];

            camera_angle = buffer[4];
            lidar_angle = buffer[5];

            // Reset the watchdog timer (500 ms)
            motor_watchdog = 10;
          }
        }

        state = WAIT_START;
        break;
    }
  }
}

// ==========================================
// APPLY VALUES TO THE HARDWARE
// ==========================================
void applyHardware() {
  M1.run(left_dir);
  M1.setSpeed(left_pwm);

  M4.run(left_dir);
  M4.setSpeed(left_pwm);

  M2.run(right_dir);
  M2.setSpeed(right_pwm);

  M3.run(right_dir);
  M3.setSpeed(right_pwm);

  cameraServo.write(camera_angle);
  lidarServo.write(lidar_angle);
}

// Arduino setup
void setup() {
  Serial.begin(115200);

  cameraServo.attach(10);
  cameraServo.write(camera_angle);

  lidarServo.attach(9);
  lidarServo.write(lidar_angle);

  pinMode(pinEncFL, INPUT_PULLUP);
  pinMode(pinEncFR, INPUT_PULLUP);
  pinMode(pinEncRL, INPUT_PULLUP);
  pinMode(pinEncRR, INPUT_PULLUP);

  enableInterrupt(pinEncFL, countFL, RISING);
  enableInterrupt(pinEncFR, countFR, RISING);
  enableInterrupt(pinEncRL, countRL, RISING);
  enableInterrupt(pinEncRR, countRR, RISING);
}

void loop() {
  unsigned long now = millis();

  // LISTEN TO THE RASPBERRY PI WITHOUT BLOCKING
  receiveMessage();

  // EXECUTE TASKS EVERY 50 ms
  if (now - last_loop_time >= 50) {
    last_loop_time = now;

    // Read encoder values while briefly disabling interrupts
    noInterrupts();
    encoder_buffer[0] = ticks_FL; ticks_FL = 0;
    encoder_buffer[1] = ticks_FR; ticks_FR = 0;
    encoder_buffer[2] = ticks_RL; ticks_RL = 0;
    encoder_buffer[3] = ticks_RR; ticks_RR = 0;
    interrupts();

    // Send odometry data
    sendMessage(ID_ENCODERS, encoder_buffer, 4);

    // Watchdog
    if (motor_watchdog > 0) {
      motor_watchdog--;
    } else {
      left_dir = RELEASE;
      left_pwm = 0;
      right_dir = RELEASE;
      right_pwm = 0;
    }

    // Update motors and servos
    applyHardware();
  }
}
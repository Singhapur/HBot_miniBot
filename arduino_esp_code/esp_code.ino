#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// --- BINARY COMMUNICATION PROTOCOL ---
#define START_BYTE 0xAA
#define END_BYTE   0x55
#define ID_IMU     0x03  // Identifier for MPU-9150 data (now including magnetometer)
#define ID_LIDAR   0x04  // Identifier for TF-Luna data

// --- HARDWARE ---
Adafruit_MPU6050 mpu;

// Create a second I2C bus for the LiDAR (same as in your previous code)
TwoWire LunaI2C = TwoWire(1);
#define I2C_SDA_LUNA 17
#define I2C_SCL_LUNA 16
#define I2C_ADDRESS_TFLUNA 0x10

// I2C command used to request a measurement from the TF-Luna
unsigned char buf1[] = { 0x5A, 0x05, 0x00, 0x01, 0x60 };

// I2C addresses of the internal chips
#define MPU_I2C_ADDR 0x68
#define MAG_I2C_ADDR 0x0C // AK8975 chip inside the MPU-9150

unsigned long last_imu_time = 0;
unsigned long last_lidar_time = 0;

// ==========================================
// MEMORY TRICK (Now with 9 floats)
// ==========================================
union ImuDataPack {
  float values[9];      // [ax, ay, az, gx, gy, gz, mx, my, mz]
  uint8_t bytes[36];    // 9 floats * 4 bytes = 36 raw bytes
};

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
// CONFIGURE BYPASS MODE FOR THE MAGNETOMETER
// ==========================================
void setupMagnetometer() {
  // Enable I2C Bypass on the MPU6050 (Register 0x37, bit 1)
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(0x37); // INT_PIN_CFG
  Wire.write(0x02); // BYPASS_EN = 1
  Wire.endTransmission();
  delay(10);
}

// ==========================================
// READ THE AK8975 MAGNETOMETER
// ==========================================
void readMagnetometer(float &mx, float &my, float &mz) {
  // Tell the AK8975 to perform a single measurement
  Wire.beginTransmission(MAG_I2C_ADDR);
  Wire.write(0x0A); // CNTL register
  Wire.write(0x01); // Single measurement mode
  Wire.endTransmission();

  // Give the sensor a couple of milliseconds to measure the magnetic field
  delay(2);

  // Read the 6 data bytes (X_L, X_H, Y_L, Y_H, Z_L, Z_H) starting from register 0x03
  Wire.beginTransmission(MAG_I2C_ADDR);
  Wire.write(0x03);
  Wire.endTransmission(false);

  Wire.requestFrom(MAG_I2C_ADDR, 6);
  if (Wire.available() >= 6) {
    int16_t x = Wire.read() | (Wire.read() << 8); // Little Endian
    int16_t y = Wire.read() | (Wire.read() << 8);
    int16_t z = Wire.read() | (Wire.read() << 8);

    // Multiply by the approximate AK8975 resolution (0.3 µT per LSB)
    mx = x * 0.3f;
    my = y * 0.3f;
    mz = z * 0.3f;
  } else {
    mx = 0;
    my = 0;
    mz = 0;
  }
}

void setup() {
  Serial.begin(115200);

  // Initialize the main I2C bus (Pins 21 and 22) for the IMU
  Wire.begin();

  // Initialize the secondary I2C bus (Pins 17 and 16) for the LiDAR
  LunaI2C.begin(I2C_SDA_LUNA, I2C_SCL_LUNA);

  if (!mpu.begin()) {
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  setupMagnetometer();
}

void loop() {
  unsigned long now = millis();

  // ==========================================
  // 1. READ AND SEND IMU + MAGNETOMETER (50 Hz)
  // ==========================================
  if (now - last_imu_time >= 20) {
    last_imu_time = now;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float mag_x, mag_y, mag_z;
    readMagnetometer(mag_x, mag_y, mag_z);

    ImuDataPack imuPack;
    imuPack.values[0] = a.acceleration.x;
    imuPack.values[1] = a.acceleration.y;
    imuPack.values[2] = a.acceleration.z;
    imuPack.values[3] = g.gyro.x;
    imuPack.values[4] = g.gyro.y;
    imuPack.values[5] = g.gyro.z;
    imuPack.values[6] = mag_x;
    imuPack.values[7] = mag_y;
    imuPack.values[8] = mag_z;

    sendMessage(ID_IMU, imuPack.bytes, 36);
  }

  // ==========================================
  // 2. READ AND SEND TF-LUNA (I2C) - Every 20 ms
  // ==========================================
  if (now - last_lidar_time >= 20) {
    last_lidar_time = now;

    // Request a measurement
    LunaI2C.beginTransmission(I2C_ADDRESS_TFLUNA);
    LunaI2C.write(buf1, 5);
    LunaI2C.endTransmission();

    // Read 9 bytes
    LunaI2C.requestFrom((uint16_t)I2C_ADDRESS_TFLUNA, (uint8_t)9);
    uint8_t data[9] = { 0 };
    int index = 0;

    while (LunaI2C.available() > 0 && index < 9) {
      data[index++] = LunaI2C.read();
    }

    // If all 9 bytes were successfully read, package and send them
    if (index == 9) {
      uint8_t lidarPack[2];
      lidarPack[0] = data[2]; // Distance low byte
      lidarPack[1] = data[3]; // Distance high byte

      sendMessage(ID_LIDAR, lidarPack, 2);
    }
  }
}
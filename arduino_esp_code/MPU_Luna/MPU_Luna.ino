#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// --- PROTOCOLO DE COMUNICACIÓN BINARIA ---
#define START_BYTE 0xAA
#define END_BYTE   0x55
#define ID_IMU     0x03  // Identificador para los datos del MPU-9150 (Ahora con Mag)
#define ID_LIDAR   0x04  // Identificador para los datos del TF-Luna

// --- HARDWARE ---
Adafruit_MPU6050 mpu;

// NUEVO: Creamos un segundo bus I2C para el LiDAR (Igual que en tu código viejo)
TwoWire LunaI2C = TwoWire(1); 
#define I2C_SDA_LUNA 17
#define I2C_SCL_LUNA 16
#define I2C_ADDRESS_TFLUNA 0x10

// Comando I2C para pedir una lectura al TF-Luna
unsigned char buf1[] = { 0x5A, 0x05, 0x00, 0x01, 0x60 };

// Direcciones I2C de los chips internos
#define MPU_I2C_ADDR 0x68
#define MAG_I2C_ADDR 0x0C // El chip AK8975 dentro del MPU-9150

unsigned long last_imu_time = 0;
unsigned long last_lidar_time = 0;

// ==========================================
// EL TRUCO DE MEMORIA (Ahora con 9 floats)
// ==========================================
union ImuDataPack {
  float values[9];      // [ax, ay, az, gx, gy, gz, mx, my, mz]
  uint8_t bytes[36];    // 9 floats * 4 bytes = 36 bytes puros
};

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
// CONFIGURAR MODO BYPASS PARA EL MAGNETÓMETRO
// ==========================================
void setupMagnetometer() {
  // 1. Activar el I2C Bypass en el MPU6050 (Registro 0x37, bit 1)
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(0x37); // INT_PIN_CFG
  Wire.write(0x02); // BYPASS_EN = 1
  Wire.endTransmission();
  delay(10);
}

// ==========================================
// LEER EL MAGNETÓMETRO AK8975
// ==========================================
void readMagnetometer(float &mx, float &my, float &mz) {
  // 1. Pedir al AK8975 que haga UNA lectura (Single Measurement Mode)
  Wire.beginTransmission(MAG_I2C_ADDR);
  Wire.write(0x0A); // Registro CNTL
  Wire.write(0x01); // Modo de medición única
  Wire.endTransmission();
  
  // Darle un par de milisegundos para que mida el campo magnético
  delay(2); 

  // 2. Leer los 6 bytes de datos (X_L, X_H, Y_L, Y_H, Z_L, Z_H) a partir del registro 0x03
  Wire.beginTransmission(MAG_I2C_ADDR);
  Wire.write(0x03); 
  Wire.endTransmission(false);
  
  Wire.requestFrom(MAG_I2C_ADDR, 6);
  if (Wire.available() >= 6) {
    int16_t x = Wire.read() | (Wire.read() << 8); // Little Endian
    int16_t y = Wire.read() | (Wire.read() << 8);
    int16_t z = Wire.read() | (Wire.read() << 8);
    
    // Multiplicamos por la resolución aproximada del AK8975 (0.3 microTeslas por LSB)
    mx = x * 0.3f;
    my = y * 0.3f;
    mz = z * 0.3f;
  } else {
    mx = 0; my = 0; mz = 0;
  }
}

void setup() {
  Serial.begin(115200);

  // 1. Iniciar bus I2C principal (Pines 21 y 22) para el IMU
  Wire.begin();
  
  // 2. Iniciar bus I2C secundario (Pines 17 y 16) para el LiDAR
  LunaI2C.begin(I2C_SDA_LUNA, I2C_SCL_LUNA);
  
  if (!mpu.begin()) {
    while (1) { delay(10); } 
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  setupMagnetometer();
}

void loop() {
  unsigned long now = millis();

  // ==========================================
  // 1. LEER Y ENVIAR IMU + MAG (A 50 Hz)
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
  // 2. LEER Y ENVIAR TF-LUNA (Por I2C) - Cada 20ms
  // ==========================================
  if (now - last_lidar_time >= 20) {
    last_lidar_time = now;

    // Pedir lectura
    LunaI2C.beginTransmission(I2C_ADDRESS_TFLUNA);
    LunaI2C.write(buf1, 5);
    LunaI2C.endTransmission();

    // Leer 9 bytes
    LunaI2C.requestFrom((uint16_t)I2C_ADDRESS_TFLUNA, (uint8_t)9);
    uint8_t data[9] = { 0 };
    int index = 0;

    while (LunaI2C.available() > 0 && index < 9) {
      data[index++] = LunaI2C.read();
    }

    // Si hemos leído bien los 9 bytes, empaquetar y enviar
    if (index == 9) {
      uint8_t lidarPack[2];
      lidarPack[0] = data[2]; // Low byte de la distancia
      lidarPack[1] = data[3]; // High byte de la distancia
      
      sendMessage(ID_LIDAR, lidarPack, 2);
    }
  }
}

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Pines I2C
#define I2C_SDA 17
#define I2C_SCL 16

// Configuración TF-Luna
#define I2C_ADDRESS_TFLUNA 0x10
#define DATA_LENGTH 9
// Command packet sent to TF-Luna to trigger a single measurement.
// 0x5A = Start byte
// 0x05 = Command ID: “take one reading”
// 0x00 = Low byte of register/count address (unused here, set to 0)
// 0x01 = Number of measurements requested (1)
// 0x60 = Checksum (0x5A + 0x05 + 0x00 + 0x01 = 0x60)
unsigned char buf1[] = { 0x5A, 0x05, 0x00, 0x01, 0x60 };

// Configuración IMU
Adafruit_MPU6050 mpu;

unsigned long ultimo_tiempo_imu = 0;

void setup() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.begin(115200);
  
  // Iniciar MPU
  if (!mpu.begin()) {
    Serial.println("Error: No se encuentra el MPU");
    while (1) { delay(10); }
  }
  
  // Configurar sensibilidad del MPU
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  // ==========================================
  // 1. LEER TF-LUNA (LiDAR)
  // ==========================================
  Wire.beginTransmission(I2C_ADDRESS_TFLUNA);
  Wire.write(buf1, 5);
  Wire.endTransmission();

  Wire.requestFrom(I2C_ADDRESS_TFLUNA, DATA_LENGTH);
  uint8_t data[DATA_LENGTH] = { 0 };
  int index = 0;

  while (Wire.available() > 0 && index < DATA_LENGTH) {
    data[index++] = Wire.read();
  }

  if (index == DATA_LENGTH) {
    uint16_t distance = data[2] + data[3] * 256;
    // Enviamos el dato con la etiqueta 'L:'
    Serial.print("L:");
    Serial.println(distance);
  }

  // ==========================================
  // 2. LEER MPU (IMU) - Cada 50ms para no saturar
  // ==========================================
  if (millis() - ultimo_tiempo_imu > 50) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Enviamos los datos con la etiqueta 'I:' separados por comas
    Serial.print("I:");
    Serial.print(a.acceleration.x); Serial.print(",");
    Serial.print(a.acceleration.y); Serial.print(",");
    Serial.print(a.acceleration.z); Serial.print(",");
    Serial.print(g.gyro.x); Serial.print(",");
    Serial.print(g.gyro.y); Serial.print(",");
    Serial.println(g.gyro.z);
    
    ultimo_tiempo_imu = millis();
  }

  delay(250);
}

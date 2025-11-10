#include <Wire.h>

// I2C pins
#define I2C_SDA 8
#define I2C_SCL 9

// Sensor addresses
#define MPU6050_ADDR 0x68
#define QMC5883L_ADDR 0x0D

// MPU6050 registers
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43

// QMC5883L registers
#define QMC5883L_DATA_OUT_X_LSB 0x00
#define QMC5883L_STATUS 0x06
#define QMC5883L_CONTROL_1 0x09
#define QMC5883L_CONTROL_2 0x0A
#define QMC5883L_SET_RESET 0x0B

// Conversion factors
// MPU6050: ±2g for accel, ±250°/s for gyro
const float ACCEL_SCALE = 9.81 / 16384.0;
const float GYRO_SCALE = (M_PI / 180.0) / 131.0;

// QMC5883L normalization
const float MAG_SCALE = 1.0 / 3000.0;

// Timing
unsigned long lastUpdate = 0;
const unsigned long UPDATE_INTERVAL = 10000;

void setup() {
  Serial.begin(115200);
  
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  
  delay(100);
  
  initMPU6050();
  initQMC5883L();
  
  delay(100);
}

void loop() {
  unsigned long currentTime = micros();
  
  if (currentTime - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = currentTime;
    
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    
    readMPU6050(ax, ay, az, gx, gy, gz);
    readQMC5883L(mx, my, mz);

    Serial.print(currentTime);
    Serial.print(",");
    Serial.print(ax, 6);
    Serial.print(",");
    Serial.print(ay, 6);
    Serial.print(",");
    Serial.print(az, 6);
    Serial.print(",");
    Serial.print(gx, 6);
    Serial.print(",");
    Serial.print(gy, 6);
    Serial.print(",");
    Serial.print(gz, 6);
    Serial.print(",");
    Serial.print(mx, 6);
    Serial.print(",");
    Serial.print(my, 6);
    Serial.print(",");
    Serial.println(mz, 6);
  }
}

void initMPU6050() {
  writeRegister(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);
  delay(10);
  
  writeRegister(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x01);
  delay(10);
  
  writeRegister(MPU6050_ADDR, 0x1C, 0x00);
  delay(10);
  
  writeRegister(MPU6050_ADDR, 0x1B, 0x00);
  delay(10);
}

void initQMC5883L() {
  writeRegister(QMC5883L_ADDR, QMC5883L_CONTROL_2, 0x80);
  delay(10);
  
  writeRegister(QMC5883L_ADDR, QMC5883L_SET_RESET, 0x01);
  delay(10);
  
  writeRegister(QMC5883L_ADDR, QMC5883L_CONTROL_1, 0x1D);
  delay(10);
}

void readMPU6050(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  
  int16_t accelX = (Wire.read() << 8) | Wire.read();
  int16_t accelY = (Wire.read() << 8) | Wire.read();
  int16_t accelZ = (Wire.read() << 8) | Wire.read();
  
  Wire.read();
  Wire.read();
  
  int16_t gyroX = (Wire.read() << 8) | Wire.read();
  int16_t gyroY = (Wire.read() << 8) | Wire.read();
  int16_t gyroZ = (Wire.read() << 8) | Wire.read();
  
  ax = accelX * ACCEL_SCALE;
  ay = accelY * ACCEL_SCALE;
  az = accelZ * ACCEL_SCALE;
  
  gx = gyroX * GYRO_SCALE;
  gy = gyroY * GYRO_SCALE;
  gz = gyroZ * GYRO_SCALE;
}

void readQMC5883L(float &mx, float &my, float &mz) {
  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(QMC5883L_STATUS);
  Wire.endTransmission(false);
  Wire.requestFrom(QMC5883L_ADDR, 1, true);
  
  uint8_t status = Wire.read();
  
  if (!(status & 0x01)) {
    return;
  }
  
  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(QMC5883L_DATA_OUT_X_LSB);
  Wire.endTransmission(false);
  Wire.requestFrom(QMC5883L_ADDR, 6, true);
  
  int16_t magX = Wire.read() | (Wire.read() << 8);
  int16_t magY = Wire.read() | (Wire.read() << 8);
  int16_t magZ = Wire.read() | (Wire.read() << 8);
  
  mx = magX * MAG_SCALE;
  my = magY * MAG_SCALE;
  mz = magZ * MAG_SCALE;
}

void writeRegister(uint8_t deviceAddr, uint8_t regAddr, uint8_t data) {
  Wire.beginTransmission(deviceAddr);
  Wire.write(regAddr);
  Wire.write(data);
  Wire.endTransmission();
}
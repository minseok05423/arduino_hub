#include <Wire.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>

// I2C pins for 9DOF sensors
#define I2C_SDA 8
#define I2C_SCL 9

// GPS pins
#define GPS_RX_PIN 20
#define GPS_TX_PIN 21

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
const float ACCEL_SCALE = 9.81 / 16384.0;
const float GYRO_SCALE = (M_PI / 180.0) / 131.0;
const float MAG_SCALE = 1.0 / 3000.0;

// GPS objects
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

// UBX message sender
void sendUBX(uint8_t msgClass, uint8_t msgID, uint8_t *payload, uint16_t payloadSize) {
  uint8_t CK_A = 0, CK_B = 0;

  gpsSerial.write(0xB5);
  gpsSerial.write(0x62);
  gpsSerial.write(msgClass);
  gpsSerial.write(msgID);
  gpsSerial.write(payloadSize & 0xFF);
  gpsSerial.write(payloadSize >> 8);

  CK_A += msgClass;
  CK_B += CK_A;
  CK_A += msgID;
  CK_B += CK_A;
  CK_A += (payloadSize & 0xFF);
  CK_B += CK_A;
  CK_A += (payloadSize >> 8);
  CK_B += CK_A;

  for (uint16_t i = 0; i < payloadSize; i++) {
    gpsSerial.write(payload[i]);
    CK_A += payload[i];
    CK_B += CK_A;
  }

  gpsSerial.write(CK_A);
  gpsSerial.write(CK_B);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("=================================");
  Serial.println("9DOF + GPS Data Logger @ 25Hz");
  Serial.println("=================================\n");

  // Initialize I2C for 9DOF sensors
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  delay(100);

  initMPU6050();
  initQMC5883L();

  // Initialize GPS
  gpsSerial.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(1000);

  Serial.println("Configuring GPS for 25Hz...");

  // Step 1: Set measurement rate to 40ms (25Hz)
  uint8_t ratePayload[] = {
    0x28, 0x00,  // measRate: 40ms = 25Hz
    0x01, 0x00,  // navRate: 1 navigation solution per measurement
    0x01, 0x00   // timeRef: GPS time
  };
  sendUBX(0x06, 0x08, ratePayload, 6);
  delay(500);

  // Step 2: Configure GPS+Galileo
  uint8_t gnssPayload[] = {
    0x00, 0x00, 0xFF, 0x02,
    0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01,
    0x02, 0x04, 0x08, 0x00, 0x01, 0x00, 0x01, 0x01
  };
  sendUBX(0x06, 0x3E, gnssPayload, 20);
  delay(500);

  // Step 3: Disable unnecessary NMEA sentences
  uint8_t disableGSV[] = {0xF0, 0x03, 0x00};
  sendUBX(0x06, 0x01, disableGSV, 3);
  delay(100);

  uint8_t disableGSA[] = {0xF0, 0x02, 0x00};
  sendUBX(0x06, 0x01, disableGSA, 3);
  delay(100);

  uint8_t disableGLL[] = {0xF0, 0x01, 0x00};
  sendUBX(0x06, 0x01, disableGLL, 3);
  delay(100);

  uint8_t disableVTG[] = {0xF0, 0x05, 0x00};
  sendUBX(0x06, 0x01, disableVTG, 3);
  delay(100);

  // Step 4: Save configuration
  uint8_t savePayload[] = {
    0x00, 0x00, 0x00, 0x00,
    0xFF, 0xFF, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00
  };
  sendUBX(0x06, 0x09, savePayload, 12);
  delay(2000);

  Serial.println("GPS Configuration complete!\n");

  // Print CSV header
  Serial.println("timestamp_us,ax,ay,az,gx,gy,gz,mx,my,mz,lat,lon,speed_kmh,altitude_m,gps_time,satellites,hdop,heading_deg");

  delay(1000);
}

void loop() {
  static unsigned long lastStatusUpdate = 0;
  static bool gpsFixAcquired = false;

  // Feed GPS parser with incoming data
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    gps.encode(c);
  }

  // Before GPS fix: Show satellite status every second
  if (!gpsFixAcquired) {
    if (millis() - lastStatusUpdate >= 1000) {
      lastStatusUpdate = millis();

      Serial.print("Waiting for GPS fix... Satellites: ");
      if (gps.satellites.isValid()) {
        Serial.print(gps.satellites.value());
      } else {
        Serial.print("0");
      }

      Serial.print(" | HDOP: ");
      if (gps.hdop.isValid()) {
        Serial.print(gps.hdop.hdop(), 1);
      } else {
        Serial.print("N/A");
      }

      Serial.print(" | Time: ");
      if (gps.time.isValid()) {
        Serial.printf("%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
      } else {
        Serial.print("N/A");
      }
      Serial.println();
    }

    // Check if we now have GPS fix
    if (gps.location.isValid()) {
      gpsFixAcquired = true;
      Serial.println("\n*** GPS FIX ACQUIRED! Starting 25Hz data output ***\n");
    }

    return;  // Don't output data yet
  }

  // After GPS fix: Output data when GPS updates (25Hz)
  if (gps.location.isUpdated()) {
    unsigned long timestamp = micros();

    // Read 9DOF sensors
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    readMPU6050(ax, ay, az, gx, gy, gz);
    readQMC5883L(mx, my, mz);

    // Output all data in CSV format
    Serial.print(timestamp);
    Serial.print(",");

    // 9DOF data
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
    Serial.print(mz, 6);
    Serial.print(",");

    // GPS data
    if (gps.location.isValid()) {
      Serial.print(gps.location.lat(), 8);
      Serial.print(",");
      Serial.print(gps.location.lng(), 8);
    } else {
      Serial.print("NaN,NaN");
    }
    Serial.print(",");

    if (gps.speed.isValid()) {
      Serial.print(gps.speed.kmph(), 2);
    } else {
      Serial.print("NaN");
    }
    Serial.print(",");

    if (gps.altitude.isValid()) {
      Serial.print(gps.altitude.meters(), 2);
    } else {
      Serial.print("NaN");
    }
    Serial.print(",");

    if (gps.time.isValid()) {
      char timeStr[16];
      sprintf(timeStr, "%02d:%02d:%02d.%02d",
              gps.time.hour(), gps.time.minute(),
              gps.time.second(), gps.time.centisecond());
      Serial.print(timeStr);
    } else {
      Serial.print("NaN");
    }
    Serial.print(",");

    if (gps.satellites.isValid()) {
      Serial.print(gps.satellites.value());
    } else {
      Serial.print("NaN");
    }
    Serial.print(",");

    if (gps.hdop.isValid()) {
      Serial.print(gps.hdop.hdop(), 2);
    } else {
      Serial.print("NaN");
    }
    Serial.print(",");

    if (gps.course.isValid()) {
      Serial.print(gps.course.deg(), 2);
    } else {
      Serial.print("NaN");
    }

    Serial.println();
  }
}

void initMPU6050() {
  writeRegister(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);
  delay(10);
  writeRegister(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x01);
  delay(10);
  writeRegister(MPU6050_ADDR, 0x1C, 0x00);  // ±2g
  delay(10);
  writeRegister(MPU6050_ADDR, 0x1B, 0x00);  // ±250°/s
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
  Wire.read(); Wire.read();  // Skip temperature
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

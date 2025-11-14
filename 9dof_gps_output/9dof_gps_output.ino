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
#define BMP280_ADDR 0x76

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

// BMP280 registers
#define BMP280_REG_TEMP_XLSB 0xFC
#define BMP280_REG_TEMP_LSB 0xFB
#define BMP280_REG_TEMP_MSB 0xFA
#define BMP280_REG_PRESS_XLSB 0xF9
#define BMP280_REG_PRESS_LSB 0xF8
#define BMP280_REG_PRESS_MSB 0xF7
#define BMP280_REG_CONTROL 0xF4
#define BMP280_REG_CONFIG 0xF5
#define BMP280_REG_CALIB 0x88

// Conversion factors
const float ACCEL_SCALE = 9.81 / 16384.0;
const float GYRO_SCALE = (M_PI / 180.0) / 131.0;
const float MAG_SCALE = 1.0 / 3000.0;

// Binary output options
// Set to 1 to send binary payload over Serial (in addition to CSV). Set to 0 to keep CSV-only.
#define ENABLE_BINARY_OUTPUT 1
// Set to 1 to print the binary payload as hex to Serial (for debugging/verification)
#define DEBUG_HEX_PRINT 0

// Payload size calculation (see encodePayloadBinary for field layout)
#define BINARY_PAYLOAD_MAX_SIZE 256

// BMP280 calibration data
struct {
  uint16_t dig_T1;
  int16_t dig_T2;
  int16_t dig_T3;
  uint16_t dig_P1;
  int16_t dig_P2;
  int16_t dig_P3;
  int16_t dig_P4;
  int16_t dig_P5;
  int16_t dig_P6;
  int16_t dig_P7;
  int16_t dig_P8;
  int16_t dig_P9;
} bmp280_calib;

int32_t t_fine;

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
  initBMP280();

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
  Serial.println("timestamp_us,ax,ay,az,gx,gy,gz,mx,my,mz,pressure_pa,temperature_c,baro_altitude_m,lat,lon,speed_kmh,altitude_m,gps_time,satellites,hdop,heading_deg");

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

    // Read all sensors
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    float pressure, temperature, baroAltitude;
    readMPU6050(ax, ay, az, gx, gy, gz);
    readQMC5883L(mx, my, mz);
    readBMP280(pressure, temperature, baroAltitude);

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

    // BMP280 data
    Serial.print(pressure, 2);
    Serial.print(",");
    Serial.print(temperature, 2);
    Serial.print(",");
    Serial.print(baroAltitude, 2);
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
    // --- Binary encoding and send (optional) ---
#if ENABLE_BINARY_OUTPUT
    {
      uint8_t payload[BINARY_PAYLOAD_MAX_SIZE];
      size_t payloadLen = 0;

      // Prepare GPS fields in simple scalar forms
      // timestamp: micros() already in 'timestamp' variable

      // speed km/h
      float speed_kmh = gps.speed.isValid() ? gps.speed.kmph() : NAN;
      // gps altitude
      float gps_alt_m = gps.altitude.isValid() ? gps.altitude.meters() : NAN;

      // gps time components
      uint8_t hh = 0, mm = 0, ss = 0, cs = 0;
      if (gps.time.isValid()) {
        hh = gps.time.hour();
        mm = gps.time.minute();
        ss = gps.time.second();
        cs = gps.time.centisecond();
      }

      // satellites
      uint8_t sats = gps.satellites.isValid() ? gps.satellites.value() : 0;

      // hdop
      float hdop_f = gps.hdop.isValid() ? gps.hdop.hdop() : NAN;

      // heading
      float heading_f = gps.course.isValid() ? gps.course.deg() : NAN;

      // Encode into payload (little-endian compact representation)
      // Layout (little-endian):
      // uint64_t timestamp_us
      // float ax,ay,az
      // float gx,gy,gz
      // float mx,my,mz
      // float pressure_pa
      // float temperature_c
      // float baro_altitude_m
      // int32_t lat (degrees * 1e7)
      // int32_t lon (degrees * 1e7)
      // uint16_t speed_kmh (x100)
      // float altitude_m
      // uint32_t gps_time_packed (hh,mm,ss,cs)
      // uint8_t satellites
      // uint16_t hdop (x100)
      // uint16_t heading_deg (x100)

      auto write_u8 = [&](uint8_t v){ payload[payloadLen++] = v; };
      auto write_u16 = [&](uint16_t v){ payload[payloadLen++] = v & 0xFF; payload[payloadLen++] = (v>>8) & 0xFF; };
      auto write_u32 = [&](uint32_t v){ for (int i=0;i<4;i++) payload[payloadLen++] = (v >> (8*i)) & 0xFF; };
      auto write_u64 = [&](uint64_t v){ for (int i=0;i<8;i++) payload[payloadLen++] = (v >> (8*i)) & 0xFF; };
      auto write_i32 = [&](int32_t v){ for (int i=0;i<4;i++) payload[payloadLen++] = (v >> (8*i)) & 0xFF; };
      auto write_f32 = [&](float f){ uint8_t b[4]; memcpy(b, &f, 4); for (int i=0;i<4;i++) payload[payloadLen++] = b[i]; };

      // timestamp
      write_u64((uint64_t)timestamp);

      // IMU
      write_f32(ax); write_f32(ay); write_f32(az);
      write_f32(gx); write_f32(gy); write_f32(gz);

      // Mag
      write_f32(mx); write_f32(my); write_f32(mz);

      // Barometer
      write_f32(pressure);
      write_f32(temperature);
      write_f32(baroAltitude);

      // Lat/Lon scaled
      int32_t lat_i = gps.location.isValid() ? (int32_t)round(gps.location.lat() * 1e7) : 0;
      int32_t lon_i = gps.location.isValid() ? (int32_t)round(gps.location.lng() * 1e7) : 0;
      write_i32(lat_i);
      write_i32(lon_i);

      // speed scaled x100
      uint16_t speed_i = isnan(speed_kmh) ? 0xFFFF : (uint16_t)constrain((int)round(speed_kmh * 100.0f), 0, 0xFFFE);
      write_u16(speed_i);

      // altitude
      write_f32(gps_alt_m);

      // gps time packed
      uint32_t time_packed = ((uint32_t)hh << 24) | ((uint32_t)mm << 16) | ((uint32_t)ss << 8) | (uint32_t)cs;
      write_u32(time_packed);

      // satellites
      write_u8(sats);

      // hdop x100
      uint16_t hdop_i = isnan(hdop_f) ? 0xFFFF : (uint16_t)constrain((int)round(hdop_f * 100.0f), 0, 0xFFFE);
      write_u16(hdop_i);

      // heading x100
      uint16_t heading_i = isnan(heading_f) ? 0xFFFF : (uint16_t)constrain((int)round(heading_f * 100.0f), 0, 0xFFFE);
      write_u16(heading_i);

      // Send a small header then payload: 0xA5 0x5A [len low][len high]
      uint8_t hdr[4];
      hdr[0] = 0xA5; hdr[1] = 0x5A; hdr[2] = payloadLen & 0xFF; hdr[3] = (payloadLen>>8) & 0xFF;
      Serial.write(hdr, 4);
      Serial.write(payload, payloadLen);

#if DEBUG_HEX_PRINT
      // Print hex for debug
      for (size_t i=0;i<payloadLen;i++) {
        if (payload[i] < 16) Serial.print('0');
        Serial.print(payload[i], HEX);
        Serial.print(' ');
      }
      Serial.println();
#endif
    }
#endif
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

void initBMP280() {
  // Read calibration data
  Wire.beginTransmission(BMP280_ADDR);
  Wire.write(BMP280_REG_CALIB);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP280_ADDR, 24, true);

  bmp280_calib.dig_T1 = Wire.read() | (Wire.read() << 8);
  bmp280_calib.dig_T2 = Wire.read() | (Wire.read() << 8);
  bmp280_calib.dig_T3 = Wire.read() | (Wire.read() << 8);
  bmp280_calib.dig_P1 = Wire.read() | (Wire.read() << 8);
  bmp280_calib.dig_P2 = Wire.read() | (Wire.read() << 8);
  bmp280_calib.dig_P3 = Wire.read() | (Wire.read() << 8);
  bmp280_calib.dig_P4 = Wire.read() | (Wire.read() << 8);
  bmp280_calib.dig_P5 = Wire.read() | (Wire.read() << 8);
  bmp280_calib.dig_P6 = Wire.read() | (Wire.read() << 8);
  bmp280_calib.dig_P7 = Wire.read() | (Wire.read() << 8);
  bmp280_calib.dig_P8 = Wire.read() | (Wire.read() << 8);
  bmp280_calib.dig_P9 = Wire.read() | (Wire.read() << 8);

  // Configure: normal mode, temp and pressure oversampling x16, filter off
  writeRegister(BMP280_ADDR, BMP280_REG_CONTROL, 0xB7);  // osrs_t=16, osrs_p=16, mode=normal
  writeRegister(BMP280_ADDR, BMP280_REG_CONFIG, 0x00);   // standby=0.5ms, filter=off
  delay(100);
}

void readBMP280(float &pressure, float &temperature, float &altitude) {
  // Read raw pressure and temperature
  Wire.beginTransmission(BMP280_ADDR);
  Wire.write(BMP280_REG_PRESS_MSB);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP280_ADDR, 6, true);

  int32_t adc_P = ((uint32_t)Wire.read() << 12) | ((uint32_t)Wire.read() << 4) | ((Wire.read() >> 4) & 0x0F);
  int32_t adc_T = ((uint32_t)Wire.read() << 12) | ((uint32_t)Wire.read() << 4) | ((Wire.read() >> 4) & 0x0F);

  // Calculate temperature (from BMP280 datasheet)
  int32_t var1, var2;
  var1 = ((((adc_T >> 3) - ((int32_t)bmp280_calib.dig_T1 << 1))) * ((int32_t)bmp280_calib.dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)bmp280_calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp280_calib.dig_T1))) >> 12) *
          ((int32_t)bmp280_calib.dig_T3)) >> 14;
  t_fine = var1 + var2;
  temperature = (float)((t_fine * 5 + 128) >> 8) / 100.0;

  // Calculate pressure (from BMP280 datasheet)
  int64_t var1_64, var2_64, p;
  var1_64 = ((int64_t)t_fine) - 128000;
  var2_64 = var1_64 * var1_64 * (int64_t)bmp280_calib.dig_P6;
  var2_64 = var2_64 + ((var1_64 * (int64_t)bmp280_calib.dig_P5) << 17);
  var2_64 = var2_64 + (((int64_t)bmp280_calib.dig_P4) << 35);
  var1_64 = ((var1_64 * var1_64 * (int64_t)bmp280_calib.dig_P3) >> 8) + ((var1_64 * (int64_t)bmp280_calib.dig_P2) << 12);
  var1_64 = (((((int64_t)1) << 47) + var1_64)) * ((int64_t)bmp280_calib.dig_P1) >> 33;

  if (var1_64 == 0) {
    pressure = 0;
    altitude = 0;
    return;
  }

  p = 1048576 - adc_P;
  p = (((p << 31) - var2_64) * 3125) / var1_64;
  var1_64 = (((int64_t)bmp280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2_64 = (((int64_t)bmp280_calib.dig_P8) * p) >> 19;
  p = ((p + var1_64 + var2_64) >> 8) + (((int64_t)bmp280_calib.dig_P7) << 4);

  pressure = (float)p / 256.0;

  // Calculate altitude using barometric formula
  // altitude = 44330 * (1 - (pressure/101325)^0.1903)
  altitude = 44330.0 * (1.0 - pow(pressure / 101325.0, 0.1903));
}

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <MPU6050.h>
#include <DFRobot_QMC5883.h>

// Define UUIDs for the service and characteristic
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// I2C pins for ESP32-C3 SuperMini
#define I2C_SDA 8
#define I2C_SCL 9

// Batch settings
#define BATCH_SIZE 5
#define SAMPLE_INTERVAL 200  // 200ms between samples (5 samples per second)

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Sensor objects
MPU6050 mpu;
DFRobot_QMC5883 compass(&Wire, QMC5883_ADDRESS);

// Structure for single sensor reading
struct SensorReading {
  uint32_t timestamp;
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float magX, magY, magZ;
  float temperature;
};

// Buffer to store batch of readings
SensorReading sensorBatch[BATCH_SIZE];
int batchIndex = 0;

// Callback class for server connection events
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Device connected");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Device disconnected");
    }
};

// Callback class for characteristic write events
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String value = pCharacteristic->getValue();
      
      if (value.length() > 0) {
        Serial.print("Received value: ");
        for (int i = 0; i < value.length(); i++) {
          Serial.print(value[i]);
        }
        Serial.println();
      }
    }
};

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println("Starting BLE Server with MPU6050 and QMC5883L...");

  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  
  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  
  if (!mpu.testConnection()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  
  // Configure MPU6050
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  
  // Initialize QMC5883L
  Serial.println("Initializing QMC5883L...");
  while (!compass.begin()) {
    Serial.println("Could not find a valid QMC5883 sensor, check wiring!");
    delay(500);
  }
  
  if(compass.isQMC()){
    Serial.println("Initialize QMC5883");
    compass.setRange(QMC5883_RANGE_2GA);
    compass.setMeasurementMode(QMC5883_CONTINOUS); 
    compass.setDataRate(QMC5883_DATARATE_50HZ);
    compass.setSamples(QMC5883_SAMPLES_8);
  }
  Serial.println("QMC5883L initialized!");

  // Create the BLE Device
  BLEDevice::init("ESP32-C3-IMU");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // Add a descriptor for notify/indicate
  pCharacteristic->addDescriptor(new BLE2902());
  
  // Set callback for write operations
  pCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();
  
  Serial.println("BLE Server is now advertising. Waiting for a client connection...");
}

void readSensors(SensorReading &reading) {
  // Read MPU6050
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Convert to proper units
  // Accelerometer: 8g range, 16-bit ADC -> LSB sensitivity = 4096 LSB/g
  reading.accelX = ax / 4096.0;  // in g
  reading.accelY = ay / 4096.0;
  reading.accelZ = az / 4096.0;
  
  // Gyroscope: 500 deg/s range, 16-bit ADC -> LSB sensitivity = 65.5 LSB/(deg/s)
  reading.gyroX = gx / 65.5;  // in deg/s
  reading.gyroY = gy / 65.5;
  reading.gyroZ = gz / 65.5;
  
  // Read temperature from MPU6050
  int16_t rawTemp = mpu.getTemperature();
  reading.temperature = (rawTemp / 340.0) + 36.53;  // MPU6050 temperature formula
  
  // Read QMC5883L
  sVector_t mag = compass.readRaw();
  reading.magX = mag.XAxis;
  reading.magY = mag.YAxis;
  reading.magZ = mag.ZAxis;
  
  // Get timestamp in milliseconds
  reading.timestamp = millis();
}

void sendBatchData() {
  // Prepare binary data packet
  // Format: 5 readings × 44 bytes each = 220 bytes total
  uint8_t data[220];
  int index = 0;
  
  Serial.println("=== Sending Batch Data ===");
  
  for (int i = 0; i < BATCH_SIZE; i++) {
    SensorReading &reading = sensorBatch[i];
    
    // Print each reading
    Serial.printf("Reading %d - Timestamp: %u ms\n", i+1, reading.timestamp);
    Serial.printf("  Accel: X=%.2f Y=%.2f Z=%.2f g\n", 
                  reading.accelX, reading.accelY, reading.accelZ);
    Serial.printf("  Gyro: X=%.2f Y=%.2f Z=%.2f deg/s\n", 
                  reading.gyroX, reading.gyroY, reading.gyroZ);
    Serial.printf("  Mag: X=%.2f Y=%.2f Z=%.2f\n", 
                  reading.magX, reading.magY, reading.magZ);
    Serial.printf("  Temp: %.2f °C\n", reading.temperature);
    
    // Add timestamp (4 bytes)
    memcpy(&data[index], &reading.timestamp, 4);
    index += 4;
    
    // Add accelerometer data (12 bytes)
    memcpy(&data[index], &reading.accelX, 4);
    index += 4;
    memcpy(&data[index], &reading.accelY, 4);
    index += 4;
    memcpy(&data[index], &reading.accelZ, 4);
    index += 4;
    
    // Add gyroscope data (12 bytes)
    memcpy(&data[index], &reading.gyroX, 4);
    index += 4;
    memcpy(&data[index], &reading.gyroY, 4);
    index += 4;
    memcpy(&data[index], &reading.gyroZ, 4);
    index += 4;
    
    // Add magnetometer data (12 bytes)
    memcpy(&data[index], &reading.magX, 4);
    index += 4;
    memcpy(&data[index], &reading.magY, 4);
    index += 4;
    memcpy(&data[index], &reading.magZ, 4);
    index += 4;
    
    // Add temperature (4 bytes)
    memcpy(&data[index], &reading.temperature, 4);
    index += 4;
  }
  
  // Send the binary data
  pCharacteristic->setValue(data, 220);
  pCharacteristic->notify();
  Serial.println("Batch data sent via BLE (220 bytes)");
  Serial.println();
}

void loop() {
  // Read sensor data and store in batch
  readSensors(sensorBatch[batchIndex]);
  
  // Increment batch index
  batchIndex++;
  
  // If batch is full, send it
  if (batchIndex >= BATCH_SIZE) {
    if (deviceConnected) {
      sendBatchData();
    } else {
      Serial.println("Device not connected, skipping batch send");
    }
    
    // Reset batch index
    batchIndex = 0;
  }
  
  // Handle disconnection
  if (!deviceConnected && oldDeviceConnected) {
    delay(200);
    pServer->startAdvertising();
    Serial.println("Start advertising again");
    oldDeviceConnected = deviceConnected;
    // Reset batch when disconnected
    batchIndex = 0;
  }
  
  // Handle connection
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
    // Reset batch when newly connected
    batchIndex = 0;
  }
  
  delay(SAMPLE_INTERVAL);  // Wait before next sample
}
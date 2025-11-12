#include <HardwareSerial.h>

HardwareSerial gpsSerial(1);

#define GPS_RX_PIN 20
#define GPS_TX_PIN 21

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
  Serial.println("Setting GPS to 25Hz Update Rate");
  Serial.println("=================================\n");
  
  gpsSerial.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(1000);
  
  Serial.println("Step 1: Configuring measurement rate to 40ms (25Hz)...");
  
  // CFG-RATE: Set measurement period to 40ms = 25Hz
  uint8_t ratePayload[] = {
    0x28, 0x00,  // measRate: 40ms (0x0028) = 25Hz
    0x01, 0x00,  // navRate: 1 navigation solution per measurement
    0x01, 0x00   // timeRef: GPS time (0x01)
  };
  
  sendUBX(0x06, 0x08, ratePayload, 6);  // CFG-RATE
  delay(500);
  
  Serial.println("Step 2: Optimizing for GPS+Galileo (best for 25Hz)...");
  
  // CFG-GNSS: Enable only GPS and Galileo for maximum update rate
  // 25Hz works best with 2 constellations
  uint8_t gnssPayload[] = {
    0x00,        // msgVer
    0x00,        // numTrkChHw (read-only)
    0xFF,        // numTrkChUse: 255 = unlimited
    0x02,        // numConfigBlocks: 2 systems
    
    // GPS Block
    0x00,        // gnssId: GPS
    0x08,        // resTrkCh: 8 channels min
    0x10,        // maxTrkCh: 16 channels max
    0x00,        // reserved1
    0x01, 0x00, 0x01, 0x01,  // flags: enable GPS with L1C/A
    
    // Galileo Block
    0x02,        // gnssId: Galileo
    0x04,        // resTrkCh: 4 channels min
    0x08,        // maxTrkCh: 8 channels max
    0x00,        // reserved1
    0x01, 0x00, 0x01, 0x01   // flags: enable Galileo with E1
  };
  
  sendUBX(0x06, 0x3E, gnssPayload, 28);  // CFG-GNSS
  delay(500);
  
  Serial.println("Step 3: Disabling unnecessary NMEA sentences for bandwidth...");
  
  // Disable all NMEA except GGA and RMC to save bandwidth
  // CFG-MSG format: [Class, ID, rate on each port...]
  
  // Disable GSV (satellite info) - uses too much bandwidth
  uint8_t disableGSV[] = {0xF0, 0x03, 0x00};  // NMEA-GxGSV
  sendUBX(0x06, 0x01, disableGSV, 3);
  delay(100);
  
  // Disable GSA (DOP and active satellites)
  uint8_t disableGSA[] = {0xF0, 0x02, 0x00};  // NMEA-GxGSA
  sendUBX(0x06, 0x01, disableGSA, 3);
  delay(100);
  
  // Disable GLL (lat/lon)
  uint8_t disableGLL[] = {0xF0, 0x01, 0x00};  // NMEA-GxGLL
  sendUBX(0x06, 0x01, disableGLL, 3);
  delay(100);
  
  // Disable VTG (track and speed)
  uint8_t disableVTG[] = {0xF0, 0x05, 0x00};  // NMEA-GxVTG
  sendUBX(0x06, 0x01, disableVTG, 3);
  delay(100);
  
  // Keep GGA and RMC enabled (they're essential)
  Serial.println("   Keeping GGA and RMC sentences enabled");
  
  Serial.println("Step 4: Saving configuration...");
  
  uint8_t savePayload[] = {
    0x00, 0x00, 0x00, 0x00,  // Clear mask
    0xFF, 0xFF, 0x00, 0x00,  // Save mask: all
    0x00, 0x00, 0x00, 0x00   // Load mask
  };
  
  sendUBX(0x06, 0x09, savePayload, 12);  // CFG-CFG
  delay(2000);  // Wait for save to complete
  
  Serial.println("\n✓ Configuration complete!");
  Serial.println("\n25Hz GPS is now active!");
  Serial.println("===================================\n");
  Serial.println("Measuring actual update rate...\n");
}

void loop() {
  static unsigned long lastNMEA = 0;
  static unsigned long sentenceCount = 0;
  static unsigned long lastReport = 0;
  static unsigned long measureStart = 0;
  static bool measuring = false;
  
  if (gpsSerial.available()) {
    char c = gpsSerial.read();
    Serial.write(c);
    
    // Count NMEA sentences starting with $
    if (c == '$') {
      sentenceCount++;
      
      if (!measuring) {
        measuring = true;
        measureStart = millis();
      }
      
      unsigned long now = millis();
      if (lastNMEA > 0) {
        unsigned long interval = now - lastNMEA;
        // Don't print every single one, too fast!
      }
      lastNMEA = now;
    }
  }
  
  // Report update rate every 5 seconds
  if (millis() - lastReport > 5000 && measuring) {
    lastReport = millis();
    unsigned long elapsed = millis() - measureStart;
    float rate = (sentenceCount * 1000.0) / elapsed;
    
    Serial.print("\n\n[Update Rate: ");
    Serial.print(rate, 1);
    Serial.print(" sentences/sec | ");
    Serial.print(sentenceCount);
    Serial.print(" total sentences in ");
    Serial.print(elapsed / 1000.0, 1);
    Serial.println(" seconds]\n");
  }
}
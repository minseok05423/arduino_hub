#include <HardwareSerial.h>

HardwareSerial gpsSerial(1);

#define GPS_RX_PIN 20
#define GPS_TX_PIN 21

// Baud rates to test
const long baudRates[] = {4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800};
const int numBaudRates = 8;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("GPS Baud Rate Detection");
  Serial.println("=======================\n");
  
  for (int i = 0; i < numBaudRates; i++) {
    Serial.print("Testing ");
    Serial.print(baudRates[i]);
    Serial.print(" baud... ");
    
    // Initialize serial at this baud rate
    gpsSerial.end();
    delay(100);
    gpsSerial.begin(baudRates[i], SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    delay(500);
    
    // Clear buffer
    while (gpsSerial.available()) {
      gpsSerial.read();
    }
    
    // Collect data for 2 seconds
    unsigned long startTime = millis();
    String testData = "";
    int nmeaCount = 0;
    int validChars = 0;
    int totalChars = 0;
    
    while (millis() - startTime < 2000) {
      if (gpsSerial.available()) {
        char c = gpsSerial.read();
        totalChars++;
        
        // Count valid ASCII printable characters
        if ((c >= 32 && c <= 126) || c == '\r' || c == '\n') {
          validChars++;
          testData += c;
          
          // Count NMEA sentences
          if (c == '$') {
            nmeaCount++;
          }
        }
      }
    }
    
    // Calculate validity score
    float validPercent = totalChars > 0 ? (100.0 * validChars / totalChars) : 0;
    
    Serial.print("Chars: ");
    Serial.print(totalChars);
    Serial.print(", Valid: ");
    Serial.print(validPercent, 1);
    Serial.print("%, NMEA: ");
    Serial.print(nmeaCount);
    
    // Check if we found valid NMEA data
    if (nmeaCount > 0 && validPercent > 90) {
      Serial.println(" ✓ FOUND!");
      Serial.println("\n==================");
      Serial.print("GPS is at ");
      Serial.print(baudRates[i]);
      Serial.println(" baud");
      Serial.println("==================\n");
      Serial.println("Sample data:");
      Serial.println(testData.substring(0, 300));  // Show first 300 chars
      
      Serial.println("\n\nContinuous output at correct baud rate:");
      while(1) {
        if (gpsSerial.available()) {
          Serial.write(gpsSerial.read());
        }
      }
    } else if (validPercent < 50) {
      Serial.println(" ✗ (garbage data)");
    } else {
      Serial.println(" ✗ (no NMEA)");
    }
  }
  
  Serial.println("\n❌ Could not detect baud rate!");
}

void loop() {
  // Empty
}
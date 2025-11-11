#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

#define GPS_RX_PIN 20
#define GPS_TX_PIN 21

unsigned long lastPrint = 0;

void setup() {
  Serial.begin(38400);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS Ready - Waiting for satellites...");
}

void loop() {
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    gps.encode(c);
  }
  
  // Print status every 2 seconds
  if (millis() - lastPrint > 2000) {
    lastPrint = millis();
    
    Serial.print("Satellites: ");
    Serial.print(gps.satellites.value());
    Serial.print(" | Chars processed: ");
    Serial.print(gps.charsProcessed());
    Serial.print(" | Fix: ");
    Serial.print(gps.location.isValid() ? "YES" : "NO");
    
    if (gps.location.isValid()) {
      Serial.print(" | Lat: ");
      Serial.print(gps.location.lat(), 7);
      Serial.print(" Lon: ");
      Serial.print(gps.location.lng(), 7);
      Serial.print(" | Alt: ");
      Serial.print(gps.altitude.meters(), 1);
      Serial.print("m");
    }
    
    Serial.println();
  }
}
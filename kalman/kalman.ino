// NEO-M8N GPS Module Authentication Check for ESP32 Super Mini
// Displays firmware version and module information

String rxBuffer = "";
bool waitingForResponse = false;

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, 20, 21);
  
  delay(2000); // Wait for GPS to initialize
  
  Serial.println("=== NEO-M8N Authentication Check ===\n");
  
  // Request firmware and hardware version
  Serial.println("Requesting MON-VER (Firmware Version)...");
  sendUBXCommand(0x0A, 0x04);
  waitingForResponse = true;
}

void loop() {
  // Read GPS data
  while (Serial1.available()) {
    char c = Serial1.read();
    
    // Print readable ASCII characters
    if (c >= 32 && c <= 126) {
      Serial.print(c);
      rxBuffer += c;
    } else {
      // Print hex for non-printable characters
      Serial.print("[0x");
      if (c < 0x10) Serial.print("0");
      Serial.print((byte)c, HEX);
      Serial.print("]");
    }
    
    // Check for version info keywords
    if (rxBuffer.indexOf("ROM") >= 0 || 
        rxBuffer.indexOf("FWVER") >= 0 ||
        rxBuffer.indexOf("PROTVER") >= 0 ||
        rxBuffer.indexOf("MOD") >= 0) {
      // Found version information
    }
    
    // Keep buffer size manageable
    if (rxBuffer.length() > 500) {
      rxBuffer = rxBuffer.substring(rxBuffer.length() - 200);
    }
  }
}

void sendUBXCommand(uint8_t msgClass, uint8_t msgID) {
  uint8_t CK_A = 0, CK_B = 0;
  
  Serial1.write(0xB5); // Sync char 1
  Serial1.write(0x62); // Sync char 2
  Serial1.write(msgClass);
  Serial1.write(msgID);
  Serial1.write(0x00); // Length LSB
  Serial1.write(0x00); // Length MSB
  
  // Calculate checksum
  CK_A = msgClass;
  CK_B = CK_A;
  CK_A += msgID;
  CK_B += CK_A;
  CK_A += 0x00;
  CK_B += CK_A;
  CK_A += 0x00;
  CK_B += CK_A;
  
  Serial1.write(CK_A);
  Serial1.write(CK_B);
}
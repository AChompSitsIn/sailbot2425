#include <Wire.h>
#include <ModbusMaster.h>
#include <SoftwareSerial.h>

// RS485 configuration
#define RX_PIN 2
#define TX_PIN 3
SoftwareSerial rs485Serial(RX_PIN, TX_PIN);
ModbusMaster node;

// I2C configuration
#define I2C_SLAVE_ADDR 0x08

// Buffer for receiving commands from Jetson
byte commandBuffer[32];
int commandLength = 0;

// Sensor data
float windDegrees = 0.0;
float sailPosition = 0.0;  // Default sail position is 0.0

// LED for I2C communication indicator
#define LED_PIN 13
bool i2cActivity = false;
unsigned long lastActivityTime = 0;

// Command IDs
#define CMD_CHANGE_POLLING_RATE 0x01
#define CMD_REQUEST_REGISTER 0x02
#define CMD_SAIL_CONTROL 0x03

void setup() {
  Serial.begin(115200);  // For debugging
  
  // Initialize LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize RS485
  rs485Serial.begin(9600);
  node.begin(2, rs485Serial);
  
  // Initialize I2C
  Wire.begin(I2C_SLAVE_ADDR);
  Wire.setClock(100000);  // Set I2C clock to 100kHz (standard speed)
  Wire.onRequest(sendDataToJetson);
  Wire.onReceive(receiveDataFromJetson);
  
  // Blink LED to indicate startup
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
  
  Serial.println("Starting wind sensor and I2C...");
}

void loop() {
  // Read wind direction
  uint8_t result = node.readHoldingRegisters(0x0000, 1);
  if (result == node.ku8MBSuccess) {
    uint16_t windDirectionRaw = node.getResponseBuffer(0);
    windDegrees = windDirectionRaw / 10.0;
    Serial.print("Wind Direction: ");
    Serial.print(windDegrees);
    Serial.println("°");
  } else {
    Serial.print("Read Error: ");
    Serial.println(result);
    rs485Serial.end();
    delay(100);
    rs485Serial.begin(9600);
  }
  
  // Turn off LED after I2C activity (visual indicator)
  if (i2cActivity && (millis() - lastActivityTime > 200)) {
    digitalWrite(LED_PIN, LOW);
    i2cActivity = false;
  }
  
  delay(500);
}

void sendDataToJetson() {
  // Create a data packet with both wind direction and sail position
  byte dataPacket[8];
  
  // Convert wind direction to bytes (first 4 bytes)
  byte* windBytes = (byte*)&windDegrees;
  dataPacket[0] = windBytes[0];
  dataPacket[1] = windBytes[1];
  dataPacket[2] = windBytes[2];
  dataPacket[3] = windBytes[3];
  
  // Convert sail position to bytes (next 4 bytes)
  byte* sailBytes = (byte*)&sailPosition;
  dataPacket[4] = sailBytes[0];
  dataPacket[5] = sailBytes[1];
  dataPacket[6] = sailBytes[2];
  dataPacket[7] = sailBytes[3];
  
  // Send the combined data
  Wire.write(dataPacket, 8);
  
  // Indicate I2C activity
  i2cActivity = true;
  lastActivityTime = millis();
  digitalWrite(LED_PIN, HIGH);
}

void receiveDataFromJetson(int byteCount) {
  commandLength = 0;
  
  while (Wire.available()) {
    if (commandLength < sizeof(commandBuffer)) {
      commandBuffer[commandLength++] = Wire.read();
    } else {
      Wire.read();  // Discard extra data
    }
  }
  
  // Indicate I2C activity
  i2cActivity = true;
  lastActivityTime = millis();
  digitalWrite(LED_PIN, HIGH);
  
  // Process received command
  if (commandLength > 0) {
    processCommands();
  }
}

void processCommands() {
  // Process commands in buffer
  if (commandLength > 0) {
    switch (commandBuffer[0]) {
      case CMD_CHANGE_POLLING_RATE:  // Change wind sensor polling rate
        if (commandLength >= 2) {
          int pollRate = commandBuffer[1] * 100;  // Convert to milliseconds
          Serial.print("Changing poll rate to: ");
          Serial.println(pollRate);
          // Implementation to change poll rate
        }
        break;
      
      case CMD_REQUEST_REGISTER:  // Request specific register from wind sensor
        if (commandLength >= 2) {
          uint16_t regAddr = commandBuffer[1];
          uint8_t result = node.readHoldingRegisters(regAddr, 1);
          if (result == node.ku8MBSuccess) {
            Serial.print("Register ");
            Serial.print(regAddr);
            Serial.print(" value: ");
            Serial.println(node.getResponseBuffer(0));
          }
        }
        break;
        
      case CMD_SAIL_CONTROL:  // Receive sail control command
        if (commandLength >= 2) {
          // Update the sail position (this will be sent back to Jetson in sendDataToJetson)
          float receivedPosition = (float)commandBuffer[1];
          Serial.print("Received sail control command: ");
          Serial.println(receivedPosition);
          
          // For now, we'll just log it, but later we would control actual hardware
          // Currently we'll keep the sailPosition at 0.0
        }
        break;
    }
    
    // Clear command buffer after processing
    commandLength = 0;
  }
}
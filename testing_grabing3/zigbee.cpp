#include "zigbee.h"

ZigBeeSensor::ZigBeeSensor(HardwareSerial& serialPort) : serial(serialPort) {
  xPos = 0;
  yPos = 0;
  lastRequestTime = 0;
}

void ZigBeeSensor::begin() {
  serial.begin(115200);
}

void ZigBeeSensor::update() {
  unsigned long currentTime = millis();
  
  // Request data at regular intervals
  if (currentTime - lastRequestTime >= REQUEST_INTERVAL) {
    lastRequestTime = currentTime;
    serial.print('?');  // Request data from ZigBee
    Serial.println("Sent data request");
  }
  
  // Process incoming data
  if (serial.available()) {
    String receivedData = serial.readStringUntil('\n');
    receivedData.trim();
    
    Serial.print("Received: ");
    Serial.println(receivedData);
    
    int firstComma = receivedData.indexOf(',');
    int secondComma = receivedData.indexOf(',', firstComma + 1);
    int thirdComma = receivedData.indexOf(',', secondComma + 1);
    
    if (firstComma != -1 && secondComma != -1 && thirdComma != -1) {
      String matchByte = receivedData.substring(0, firstComma);
      String xCoordStr = receivedData.substring(firstComma + 1, secondComma);
      String yCoordStr = receivedData.substring(secondComma + 1, thirdComma);
      
      xPos = xCoordStr.toFloat();
      yPos = yCoordStr.toFloat();
      
      Serial.print("Updated Position: X=");
      Serial.print(xPos);
      Serial.print(" Y=");
      Serial.println(yPos);
    }
  }
}

float ZigBeeSensor::getX() {
  return xPos;
}

float ZigBeeSensor::getY() {
  return yPos;
}
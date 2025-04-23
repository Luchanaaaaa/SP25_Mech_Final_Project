#ifndef ZIGBEE_H
#define ZIGBEE_H

#include <Arduino.h>

class ZigBeeSensor {
public:
  ZigBeeSensor(HardwareSerial& serial);
  void begin();
  void update();
  float getX();
  float getY();

private:
  HardwareSerial& serial;
  float xPos;
  float yPos;
  unsigned long lastRequestTime;
  const unsigned long REQUEST_INTERVAL = 500;  // ms
};

#endif
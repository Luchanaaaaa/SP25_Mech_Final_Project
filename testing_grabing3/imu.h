#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

class IMUSensor {
public:
  IMUSensor();
  bool begin();
  float getCurrentAngle();

private:
  Adafruit_BNO055 bno;
};

#endif
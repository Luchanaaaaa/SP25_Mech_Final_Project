#ifndef IR_H
#define IR_H

#include <Arduino.h>

/**
 * IRSensor Class
 * 
 * Manages the GP2Y0A21YK0F IR sensor by SHARP
 */
class IRSensor {
public:
  // Constructor: Pin must be analog
  // offset: calibrate sensor
  IRSensor(uint8_t pin, double off = 0.0);

  // Initialize the IR sensor
  void begin();

  // Return true if the analog reading from the sensor
  // exceeds the provided threshold
  bool overThreshold(double threshold);

private:
  uint8_t _pin;
  double offset;
};

#endif

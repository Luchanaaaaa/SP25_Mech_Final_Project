#ifndef PING_H
#define PING_H

#include <Arduino.h>

/**
 * PingSensor Class
 *
 * This class handles a 3-pin ultrasonic sensor
 */
class PingSensor {
public:
  /**
   * Constructor
   * @param pin The single signal pin
   */
  PingSensor(uint8_t pin);

  /**
   * Initialize the sensor.
   */
  void begin();

  /**
   * Read and return the distance in centimeters.
   * Returns -1 if there's a timeout or no valid reading.
   */
  float getDistanceCm();

private:
  uint8_t _pin;
};

#endif

#include "ir.h"

// Constructor: set the pin
IRSensor::IRSensor(uint8_t pin, double off): _pin(pin), offset(off) {}

// Initialize the IR sensor
void IRSensor::begin() {
  pinMode(_pin, INPUT);
}

// Check if the sensor reading is over the threshold
bool IRSensor::overThreshold(double threshold) {
  threshold += offset;
  int sensorValue = analogRead(_pin);
  return (sensorValue > threshold);
}

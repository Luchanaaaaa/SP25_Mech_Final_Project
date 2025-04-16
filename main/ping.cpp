#include "ping.h"

PingSensor::PingSensor(uint8_t pin)
  : _pin(pin)
{
}

void PingSensor::begin() {
  // set the pin to LOW at startup.
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
  delay(20); // small delay for sensor stabilization
}

float PingSensor::getDistanceCm() {
  // Set the pin as OUTPUT
  pinMode(_pin, OUTPUT);
  
  // Ensure it's low
  digitalWrite(_pin, LOW);
  delayMicroseconds(2);

  // Send a short pulse (5 us)
  digitalWrite(_pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(_pin, LOW);

  // Switch pin to INPUT to read the echo
  pinMode(_pin, INPUT);

  // Measure the length of time pin stays HIGH with pulseIn
  unsigned long duration = pulseIn(_pin, HIGH);
  // If duration == 0, we timed out => no valid reading
  if (duration == 0) {
    return -1.0;
  }
  // Convert time (in microseconds) to distance in cm
  double distance = (double)duration * 343 / 10000 / 2;
  return distance;
}

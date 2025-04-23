#include "imu.h"

IMUSensor::IMUSensor() : bno(55) {
  // Constructor
}

bool IMUSensor::begin() {
  if (!bno.begin()) {
    return false;
  }
  bno.setExtCrystalUse(true);
  return true;
}

float IMUSensor::getCurrentAngle() {
  sensors_event_t event;
  bno.getEvent(&event);
  return event.orientation.x;
}
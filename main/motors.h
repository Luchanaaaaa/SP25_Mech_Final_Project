#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include <DualMAX14870MotorShield.h>  // Pololu motor driver library

class Motor {
public:
  // Constructor
  Motor();

  // Initialize motor driver
  void begin();

  // Basic motor movement functions
  void moveForward();
  void turnLeft();
  void turnRight();
  void turnAround();
  void moveForwardLeft();
  void moveForwardRight();
  void stop();


private:
  DualMAX14870MotorShield motors;  // Motor driver object
  const int SPEED = 150;           // Default base speed

  // Low-level function to directly set motor speeds
  void setMotors(int targetM1, int targetM2);
};

#endif

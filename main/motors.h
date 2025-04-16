#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include <DualMAX14870MotorShield.h>  // Include Pololu library

class Motor {
public:
  // Constructor
  Motor();

  // Initialize motors
  void begin();

  // Basic motor actions
  void moveForward();
  void turnLeft();
  void turnRight();
  void turnAround();
  void moveForwardLeft();
  void moveForwardRight();
  void stop();

  // Print encoder counts
  void printEncoders();

private:
  DualMAX14870MotorShield motors; // Pololu motor driver object
  const int SPEED = 150;          // Constant target speed for normal operation

  // Change motor speeds from the current value to the target value.
  void setMotors(int targetM1, int targetM2);
};

#endif

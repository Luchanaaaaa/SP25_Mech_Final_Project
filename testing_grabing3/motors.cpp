#include "motors.h"

Motor::Motor() {}

void Motor::begin() {
  motors.init();  // Initialize Pololu motor driver
}

// Set motor speeds directly
void Motor::setMotors(int targetM1, int targetM2) {
  motors.setM1Speed(constrain(targetM1, -400, 400));
  motors.setM2Speed(constrain(targetM2, -400, 400));
}

void Motor::moveForward() {
  setMotors(SPEED, SPEED);
}

void Motor::turnLeft() {
  setMotors(-SPEED, SPEED);
  delay(400);  // tune this delay to match ~90° rotation
  stop();
}

void Motor::turnRight() {
  setMotors(SPEED, -SPEED);
  delay(400);  // tune this delay to match ~90° rotation
  stop();
}

void Motor::turnAround() {
  setMotors(-SPEED, SPEED);
  delay(800);  // tune this for ~180° spin
  stop();
}

void Motor::moveForwardLeft() {
  setMotors(SPEED / 2, SPEED);
}

void Motor::moveForwardRight() {
  setMotors(SPEED, SPEED / 2);
}

void Motor::stop() {
  setMotors(0, 0);
}

// Placeholder (not used if encoders are disabled)
void Motor::printEncoders() {
  Serial.println("Encoders not used.");
}

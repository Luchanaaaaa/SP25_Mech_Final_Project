// ======= Includes =======
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <DualMAX14870MotorShield.h>

// ======= Sensor & Motor Setup =======
DualMAX14870MotorShield motors;
Adafruit_BNO055 bno = Adafruit_BNO055(55);

const int pingPin = 3;
unsigned long pulseduration = 0;
int distance = 0;

// ======= PID & Target Angle =======
double targetAngle = 0;
double Kp = 3.5;

// ======= Setup =======
void setup() {
  Serial.begin(9600);
  delay(1000);  // Allow IMU to stabilize
  motors.enableDrivers();

  pinMode(pingPin, OUTPUT);

  if (!bno.begin()) {
    Serial.println("⚠️ IMU not detected. Check wiring.");
    while (1);
  }

  bno.setExtCrystalUse(true);
  Serial.println("✅ Robot ready!");
}

// ======= Main Loop =======
void loop() {
  measureDistance();
  distance = pulseduration * 0.0343 / 2;

  Serial.print("Distance: "); Serial.print(distance); Serial.println(" cm");

  if (distance > 20) {
    stline();  // Move straight with IMU correction
  } else {
    stopMovement();
    delay(500);
    driftLeft();  // You can swap with driftRight() or driftUTurn()
  }

  delay(100);  // Loop delay
}

// ======= Straight Line Motion =======
void stline() {
  double currentAngle = getCurrentAngle();
  double error = angleError(targetAngle, currentAngle);
  double correction = Kp * error;

  int baseSpeed = 200;
  int leftSpeed = constrain(baseSpeed - correction, 75, 400);
  int rightSpeed = constrain(baseSpeed + correction, 75, 400);

  motors.setM1Speed(rightSpeed);
  motors.setM2Speed(leftSpeed);
}

// ======= Ping Sensor =======
void measureDistance() {
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(5);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  pinMode(pingPin, INPUT);
  pulseduration = pulseIn(pingPin, HIGH);
}

// ======= IMU Heading =======
double getCurrentAngle() {
  sensors_event_t event;
  bno.getEvent(&event);
  return event.orientation.x;
}

double angleError(double target, double current) {
  double error = target - current;
  if (error > 180) error -= 360;
  if (error < -180) error += 360;
  return error;
}

double normalizeAngle(double angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}

// ======= Drift Turn Functions =======
void driftLeft(int speed = 150) {
  double current = getCurrentAngle();
  double target = normalizeAngle(current - 90);
  motors.setM1Speed(speed);
  motors.setM2Speed(-speed);
  while (angleError(target, getCurrentAngle()) > 2) delay(5);
  stopMovement();
}

void driftRight(int speed = 150) {
  double current = getCurrentAngle();
  double target = normalizeAngle(current + 90);
  motors.setM1Speed(-speed);
  motors.setM2Speed(speed);
  while (angleError(target, getCurrentAngle()) > 2) delay(5);
  stopMovement();
}

void driftUTurn(int speed = 150) {
  double current = getCurrentAngle();
  double target = normalizeAngle(current + 180);
  motors.setM1Speed(-speed);
  motors.setM2Speed(speed);
  while (angleError(target, getCurrentAngle()) > 2) delay(5);
  stopMovement();
}

// ======= Stop =======
void stopMovement() {
  motors.setM1Speed(0);
  motors.setM2Speed(0);
}

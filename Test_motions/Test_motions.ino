// ======= Includes =======
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <DualMAX14870MotorShield.h>

// ======= Sensor & Motor Setup =======
DualMAX14870MotorShield motors;
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// ======= Ping Sensor =======
const int pingPin = 3;
unsigned long pulseduration = 0;
int distance = 0;

// ======= PID & Target Angle =======
double targetAngle = 0;
double Kp = 3.5;

// ======= Function Declarations =======
void driftLeft(int speed = 150);
void driftRight(int speed = 150);
void driftUTurn(int speed = 150);
void stopMovement();
void measureDistance();
void stline();
double getCurrentAngle();
double angleError(double target, double current);
double normalizeAngle(double angle);

// ======= Setup =======
void setup() {
  Serial.begin(9600);
  delay(1000);
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
  // Optional: check starting distance
  measureDistance();
  distance = pulseduration * 0.0343 / 2;
  Serial.print("Start Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Step 1: Go straight
  Serial.println("➡️ Moving straight");
  for (int i = 0; i < 30; i++) {
    stline();
    delay(100);
  }
  stopMovement();
  delay(500);

  // Step 2: Turn Left
  Serial.println("↩️ Drift Left");
  driftRight();
  targetAngle = normalizeAngle(targetAngle - 90);
  delay(500);

  // Step 3: Go straight
  Serial.println("➡️ Moving straight");
  for (int i = 0; i < 30; i++) {
    stline();
    delay(100);
  }
  stopMovement();
  delay(500);

  // Step 4: Turn Right
  Serial.println("↪️ Drift Right");
  driftRight();
  targetAngle = normalizeAngle(targetAngle - 90);
  delay(500);

  // Step 5: Go straight
  Serial.println("➡️ Moving straight");
  for (int i = 0; i < 100; i++) {
    stline();
    delay(100);
  }
  stopMovement();
  delay(500);

  // Step 6: U-Turn
  Serial.println("🔄 Drift U-Turn");
  driftUTurn();
  targetAngle = normalizeAngle(targetAngle + 180);
  delay(500);

  // Step 7: Final straight
  Serial.println("⬅️ Returning straight");
  for (int i = 0; i < 30; i++) {
    stline();
    delay(100);
  }
  stopMovement();
  delay(1000);

  Serial.println("✅ Sequence complete!");
  while (1);  // Halt forever
}

// ======= Straight Line Motion with Obstacle Detection =======
void stline() {
  measureDistance();
  distance = pulseduration * 0.0343 / 2;

  if (distance <= 20 && distance > 0) {
    stopMovement();
    Serial.println("🛑 Obstacle detected! Stopping.");
    while (true);  // Halt
  }

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
void driftLeft(int speed) {
  double current = getCurrentAngle();
  double target = normalizeAngle(current - 90);
  motors.setM1Speed(speed);
  motors.setM2Speed(-speed);
  while (angleError(target, getCurrentAngle()) > 2) delay(5);
  stopMovement();
}

void driftRight(int speed) {
  double current = getCurrentAngle();
  double target = normalizeAngle(current + 90);
  motors.setM1Speed(-speed);
  motors.setM2Speed(speed);
  while (angleError(target, getCurrentAngle()) > 2) delay(5);
  stopMovement();
}

void driftUTurn(int speed) {
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

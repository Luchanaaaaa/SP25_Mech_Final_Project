// ======= Includes =======
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <DualMAX14870MotorShield.h>
#include <Pixy2.h>

// ======= Hardware Setup =======
DualMAX14870MotorShield motors;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Pixy2 pixy;

// ======= FSM States =======
enum RobotState {
  SEARCH_PUCK,
  GO_TO_THE_PUCK
};
RobotState currentState = SEARCH_PUCK;

// ======= PID Control =======
double targetAngle = 0;
double Kp = 3.5;

// ======= Setup =======
void setup() {
  Serial.begin(9600);
  delay(1000);
  
  motors.enableDrivers();
  if (!bno.begin()) {
    Serial.println("❌ IMU not detected.");
    while (1);
  }
  bno.setExtCrystalUse(true);

  pixy.init();

  Serial.println("✅ Robot ready (Pixy + IMU + FSM)");
}

// ======= Main FSM Loop =======
void loop() {
  switch (currentState) {

    case SEARCH_PUCK:
      searchPuck();
      if (pixySeesOrange()) {
        currentState = GO_TO_THE_PUCK;
      }
      break;

    case GO_TO_THE_PUCK:
      if (pixySeesOrange()) {
        Go_TO_THE_PUCK();
      } else {
        currentState = SEARCH_PUCK;
      }
      break;
  }
}

// ======= Function: Pixy Sees Orange Puck =======
bool pixySeesOrange() {
  pixy.ccc.getBlocks();
  return (pixy.ccc.numBlocks > 0 && pixy.ccc.blocks[0].m_signature == 1);
}

// ======= Function: Search for Puck =======
void searchPuck() {
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks > 0 && pixy.ccc.blocks[0].m_signature == 1) {
    // Puck found → stop rotating
    motors.setM1Speed(0);
    motors.setM2Speed(0);
    Serial.println("Puck detected.");
  } else {
    // Rotate slowly to search
    motors.setM1Speed(100);
    motors.setM2Speed(-100);
    Serial.println("🔍 Searching for puck...");
  }
}

// ======= Function: Go to the Puck with IMU + Pixy Y tracking =======
// ======= Function: Go to the Puck with IMU + Pixy Y tracking =======
void Go_TO_THE_PUCK() {
  pixy.ccc.getBlocks();

  // Check if the puck (signature 1) is visible
  if (pixy.ccc.numBlocks == 0 || pixy.ccc.blocks[0].m_signature != 1) {
    stopMovement();
    Serial.println("❌ Puck lost.");
    return;
  }

  // Get X and Y positions of the puck
  int x = pixy.ccc.blocks[0].m_x;  // Horizontal (left-right)
  int y = pixy.ccc.blocks[0].m_y;  // Vertical (distance)

  // Speed scaling based on distance (Y)
  int baseSpeed;
  if (y < 100) {
    baseSpeed = 400;  // Far → go faster
  } else {
    baseSpeed = 200;  // Close → slow down
  }

  // Steering correction based on X position
  int xCenter = 160;          // Center of Pixy frame
  int xError = x - xCenter;   // -ve = puck left, +ve = puck right
  int correction = xError / 2; // Adjust gain as needed

  // Calculate final motor speeds
  int leftSpeed = constrain(baseSpeed - correction, 75, 400);
  int rightSpeed = constrain(baseSpeed + correction, 75, 400);

  // Apply speeds to motors
  motors.setM1Speed(rightSpeed);  // Right motor
  motors.setM2Speed(leftSpeed);   // Left motor

  // Debug output
  Serial.print("🧡 Puck X: ");
  Serial.print(x);
  Serial.print(" | Y: ");
  Serial.print(y);
  Serial.print(" | BaseSpeed: ");
  Serial.print(baseSpeed);
  Serial.print(" | Correction: ");
  Serial.println(correction);
}

// ======= IMU Helper Functions =======
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

// ======= Motor Stop =======
void stopMovement() {
  motors.setM1Speed(0);
  motors.setM2Speed(0);
}

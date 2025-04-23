/*********************************************************************
 *  JHockey 2025
 *  ---------------------------------------------------------------
 *  Sensors:
 *    - Pixy2     : detect orange puck
 *    - Ping      : verify puck is in front of robot
 *    - BNO055    : IMU for heading / PID
 *    - ZigBee    : receives position data
 *
 *  STATE FLOW:
 *    SEARCH_PUCK -> GO_TO_PUCK -> ALIGN_ROBOT -> GO_TO_GOAL -> STOP
 *                      |              |
 *                      v              v
 *                   SEARCH_PUCK  SEARCH_PUCK
 *********************************************************************/

#include "motors.h"
#include "ping.h"
#include "pixy.h"
#include "imu.h"
#include "zigbee.h"

// -------------------- Hardware objects ----------------------------
#define PING_PIN     3
#define XBEE_SERIAL  Serial1
#define GOAL_X       55.0
#define GOAL_Y       223.0

Motor           motor;
PingSensor      ping(PING_PIN);
PixySensor      pixy;
IMUSensor       imu;
ZigBeeSensor    zigbee(XBEE_SERIAL);

// -------------------- FSM States ----------------------------------
enum RobotState {
  SEARCH_PUCK,
  GO_TO_PUCK,
  ALIGN_ROBOT,
  GO_TO_GOAL,
  SET_ANGLE,
  STRIKE,
  STOP
};
RobotState currentState = SEARCH_PUCK;

// -------------------- Constants -----------------------------------
const int   PIXY_CENTER_X     = 160;
const float PIXY_CORRECTION   = 0.5;  // correction factor for steering
const int   BASE_SPEED        = 200;  // default movement speed
const float PID_KP            = 3.5;  // proportional control constant
const float ANGLE_TOLERANCE   = 60.0; // degrees
const float PUCK_DISTANCE_MIN = 2.0;  // cm
const float PUCK_DISTANCE_MAX = 6.0;  // cm

// -------------------- Position Data -------------------------------
float robotX = 0.0;
float robotY = 0.0;
int n = 1;  // strike counter

// -------------------- Function Prototypes -------------------------
void searchPuck();
void goToPuck();
void alignToGoal();
void goToGoal();
void setAngle();
void strikePuck();
void updateRobotPosition();
bool puckInPossession();
float calculateGoalAngle();
float getAngleError(float target, float current);

// -------------------- Setup ---------------------------------------
void setup() {
  Serial.begin(115200);
  
  // Initialize hardware components
  motor.begin();
  ping.begin();
  pixy.begin();
  if (!imu.begin()) {
    Serial.println("❌ IMU not detected.");
    while (1);
  }
  zigbee.begin();
  
  Serial.println("✅ Robot ready (Pixy + FSM + Ping)");
  
  // Initial state decision
  currentState = (n == 0) ? SET_ANGLE : SEARCH_PUCK;
}

// -------------------- Main FSM Loop -------------------------------
void loop() {
  updateRobotPosition();
  
  switch (currentState) {
    case SEARCH_PUCK:
      searchPuck();
      if (pixy.seesPuck()) currentState = GO_TO_PUCK;
      break;
      
    case GO_TO_PUCK:
      if (pixy.seesPuck()) {
        goToPuck();
      } else {
        currentState = SEARCH_PUCK;
      }
      
      if (puckInPossession()) {
        Serial.println("✅ Puck acquired! Aligning to goal...");
        currentState = ALIGN_ROBOT;
      }
      break;
      
    case ALIGN_ROBOT:
      if (!puckInPossession()) {
        Serial.println("❌ Lost puck during alignment — back to search.");
        currentState = SEARCH_PUCK;
        break;
      }
      
      alignToGoal();
      break;
      
    case GO_TO_GOAL:
      if (puckInPossession()) {
        goToGoal();
      } else {
        Serial.println("❌ Lost puck — back to search.");
        currentState = SEARCH_PUCK;
      }
      break;
      
    case SET_ANGLE:
      setAngle();
      currentState = STRIKE;
      break;
      
    case STRIKE:
      strikePuck();
      currentState = SEARCH_PUCK;
      n++;
      break;
      
    case STOP:
      motor.stop();
      break;
  }
}

// -------------------- FSM State Functions ------------------------

void searchPuck() {
  if (pixy.seesPuck()) {
    motor.stop();
    Serial.println("🧡 Puck detected.");
  } else {
    motor.rotate(100);
    Serial.println("🔍 Searching for puck...");
  }
}

void goToPuck() {
  if (!pixy.seesPuck()) {
    motor.stop();
    Serial.println("❌ Puck lost.");
    return;
  }

  int x = pixy.getPuckX();
  int xError = x - PIXY_CENTER_X;
  int correction = xError * PIXY_CORRECTION;
  
  motor.moveWithCorrection(BASE_SPEED, correction);
  
  Serial.print("🧡 Puck X: ");
  Serial.print(x);
  Serial.print(" | Correction: ");
  Serial.println(correction);
  Serial.print("Ping: ");
  Serial.println(ping.getDistance());
}

void alignToGoal() {
  float goalAngle = calculateGoalAngle();
  float currentAngle = imu.getCurrentAngle();
  float error = getAngleError(goalAngle, currentAngle);
  
  Serial.print("🔁 Aligning | Current: ");
  Serial.print(currentAngle);
  Serial.print(" | Target: ");
  Serial.print(goalAngle);
  Serial.print(" | Error: ");
  Serial.println(error);
  
  if (abs(error) <= ANGLE_TOLERANCE) {
    Serial.println("✅ Aligned to goal — moving to GO_TO_GOAL.");
    currentState = GO_TO_GOAL;
  } else {
    float correction = PID_KP * error;
    motor.moveWithCorrection(150, correction);
  }
}

void goToGoal() {
  float goalAngle = calculateGoalAngle();
  float currentAngle = imu.getCurrentAngle();
  float error = getAngleError(goalAngle, currentAngle);
  float correction = PID_KP * error;
  
  motor.moveWithCorrection(300, correction);
  
  Serial.print("🎯 Goal Heading: ");
  Serial.print(goalAngle);
  Serial.print(" | Current: ");
  Serial.print(currentAngle);
  Serial.print(" | Error: ");
  Serial.println(error);
}

void setAngle() {
  Serial.println("🎯 Quick angle + strike...");
  motor.rotate(150);
  delay(120);
  motor.moveForward(300);
  delay(400);
  motor.stop();
  Serial.println("✅ Angled strike complete.");
}

void strikePuck() {
  if (!pixy.seesPuck()) {
    Serial.println("🔍 Searching for puck...");
    motor.rotate(100);
    return;
  }
  
  int x = pixy.getPuckX();
  int correction = (x - PIXY_CENTER_X) * PIXY_CORRECTION;
  motor.moveWithCorrection(300, correction);
  delay(400);
  motor.stop();
  Serial.println("💥 Puck struck – move complete.");
}

// -------------------- Helper Functions ---------------------------

void updateRobotPosition() {
  zigbee.update();
  robotX = zigbee.getX();
  robotY = zigbee.getY();
}

bool puckInPossession() {
  float dist = ping.getDistance();
  return (dist > PUCK_DISTANCE_MIN && dist < PUCK_DISTANCE_MAX);
}

float calculateGoalAngle() {
  float goalAngleRad = atan2(GOAL_Y - robotY, GOAL_X - robotX);
  return goalAngleRad * 180 / PI;
}

float getAngleError(float target, float current) {
  float error = target - current;
  if (error > 180) error -= 360;
  if (error < -180) error += 360;
  return error;
}
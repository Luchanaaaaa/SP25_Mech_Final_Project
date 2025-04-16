/*********************************************************************
 *  JHockey 2025
 *  ---------------------------------------------------------------
 *  Sensors:
 *    - Pixy2     : detect orange puck
 *    - Ping      : verify puck is in front of robot
 *    - BNO055    : IMU for heading / PID
 *    - ZigBee    : receives "M,TTTT,XXX,YYY" – we only use X,Y,match byte
 *
 *  STATE FLOW:
 *    FIND_PUCK  ->  ALIGN_TO_PUCK  ->  GO_TO_PUCK
 *                                       |  (lost)
 *                                       v
 *                                    FIND_PUCK
 *    GO_TO_PUCK -> PUSH_TO_GOAL -> FINISHED
 *********************************************************************/

#include "motors.h"
#include "ping.h"
#include "pixy.h"
#include "ir.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// -------------------- Hardware objects ----------------------------
#define PING_PIN     2
#define XBEE_SERIAL  Serial2
#define LEFT_IR_PIN  A0
#define RIGHT_IR_PIN A1

Motor           motor;
PingSensor      pingSense(PING_PIN);
PixySensor      pixy;
Adafruit_BNO055 bno(55, 0x28);
IRSensor        leftIR(LEFT_IR_PIN);
IRSensor        rightIR(RIGHT_IR_PIN);

// -------------------- FSM States ----------------------------------
enum State {
  FIND_PUCK,
  ALIGN_TO_PUCK,
  GO_TO_PUCK,
  PUSH_TO_GOAL,
  FINISHED
};
State currentState = FIND_PUCK;

// -------------------- Constants -----------------------------------
const int   PIXY_CENTER_TOL  = 10;
const int   PIXY_SIZE_CLOSE  = 1200;
const float PING_HIT_CM      = 25.0;
const double IR_THRESHOLD    = 200.0;
const int   GOAL_X           = 0;
const int   GOAL_Y           = 200;
const int   GOAL_RADIUS_CM   = 20;

// -------------------- ZigBee Data ---------------------------------
bool  matchOn   = true; // change to false if waiting for match start
long  matchTime = 0;
float myX = -1, myY = -1;

// -------------------- Function Prototypes -------------------------
bool  pixySeesPuck();
bool  pixyCentered();
void  rotateScan();
void  alignToPixy();
void  driveTowardsPixy();
bool  pingHasPuck();
void  maintainHeadingToGoal();
bool  lostPuck();
bool  atGoal();
void  parseZigBee();

// -------------------- IR-based Obstacle Avoidance -----------------
bool avoidObstacleWithIR() {
  if (leftIR.overThreshold(IR_THRESHOLD)) {
    motor.moveForwardRight();  // Steer away from left wall
    return true;
  } else if (rightIR.overThreshold(IR_THRESHOLD)) {
    motor.moveForwardLeft();   // Steer away from right wall
    return true;
  }
  return false;
}

// -------------------- Setup ---------------------------------------
void setup() {
  Serial.begin(115200);
  XBEE_SERIAL.begin(9600);

  motor.begin();
  pingSense.begin();
  pixy.begin();
  leftIR.begin();
  rightIR.begin();

  if (!bno.begin()) {
    Serial.println("BNO055 not detected – halting.");
    while (1);
  }
}

// -------------------- Main FSM Loop -------------------------------
void loop() {
  parseZigBee();

  switch (currentState) {

    // 1) Scan until puck is found
    case FIND_PUCK:
      rotateScan();
      if (pixySeesPuck()) currentState = ALIGN_TO_PUCK;
      break;

    // 2) Align robot to face puck
    case ALIGN_TO_PUCK:
      if (avoidObstacleWithIR()) break;
      alignToPixy();
      if (!pixySeesPuck())         currentState = FIND_PUCK;
      else if (pixyCentered())     currentState = GO_TO_PUCK;
      break;

    // 3) Drive toward the puck
    case GO_TO_PUCK:
      if (avoidObstacleWithIR()) break;
      driveTowardsPixy();
      if (lostPuck())              currentState = FIND_PUCK;
      else if (pingHasPuck())      currentState = PUSH_TO_GOAL;
      break;

    // 4) Push puck toward goal
    case PUSH_TO_GOAL:
      if (avoidObstacleWithIR()) break;
      maintainHeadingToGoal();
      if (lostPuck())              currentState = FIND_PUCK;
      else if (atGoal())           currentState = FINISHED;
      break;

    // 5) Celebration / stop
    case FINISHED:
      motor.stop();
      break;
  }
}

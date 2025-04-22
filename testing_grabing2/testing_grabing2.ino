// ======= Includes =======
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <DualMAX14870MotorShield.h>
#include <Pixy2.h>

// ======= Pins & Constants =======
const int pingPin = 3;
const float goalX = 55.0;
const float goalY = 223.0;
float robotX = 0.0;
float robotY = 0.0;
int n = 1;
int baseSpeed =200;

// ======= Hardware Setup =======
DualMAX14870MotorShield motors;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Pixy2 pixy;

// ======= FSM States =======
enum RobotState {
  SET_ANGLE,
  STRIKE,
  SEARCH_PUCK,
  GO_TO_THE_PUCK,
  ALIGN_ROBOT,
  GO_TO_GOAL,
  STOP
};
RobotState currentState;

// ======= PID Constant =======
double Kp = 3.5;

// ======= Setup =======
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  delay(1000);

  motors.enableDrivers();
  if (!bno.begin()) {
    Serial.println("❌ IMU not detected.");
    while (1);
  }
  bno.setExtCrystalUse(true);
  pixy.init();

  Serial.println("✅ Robot ready (Pixy + FSM + Ping)");
  currentState = (n == 0) ? SET_ANGLE : SEARCH_PUCK;
}

// ======= FSM LOOP =======
void loop() {
  updateRobotPositionFromZigbee();
  switch (currentState) {
    

    case SET_ANGLE:
      angleStrike();
      currentState = STRIKE;
      break;

    case STRIKE:
      strikePuck();
      currentState = SEARCH_PUCK;
      n++;
      break;

    case SEARCH_PUCK:
      searchPuck();
      if (pixySeesOrange()) currentState = GO_TO_THE_PUCK;
      break;

    case GO_TO_THE_PUCK:
      if (pixySeesOrange()) {
        Go_TO_THE_PUCK();
      }
      else {currentState = SEARCH_PUCK;
      }
      break;
  
      

    case ALIGN_ROBOT:
      if (!puckInPossession()) {
        Serial.println("❌ Lost puck during alignment — back to search.");
        currentState = SEARCH_PUCK;
        break;
      }
      

      float goalAngleRad = atan2(goalY - robotY, goalX - robotX);
      float goalAngleDeg = goalAngleRad * 180 / PI;
      float currentAngle = getCurrentAngle();
      float error = angleError(goalAngleDeg, currentAngle);

      Serial.print("🔁 Aligning | Current: ");
      Serial.print(currentAngle);
      Serial.print(" | Target: ");
      Serial.print(goalAngleDeg);
      Serial.print(" | Error: ");
      Serial.println(error);

      if (error >= -60 && error <= 60) {
        Serial.println("✅ Aligned to goal — moving to GO_TO_GOAL.");
        currentState = GO_TO_GOAL;
      } else {
        float correction = Kp * error;
        int leftSpeed = constrain(150 - correction, 75, 400);
        int rightSpeed = constrain(150 + correction, 75, 400);
        motors.setM1Speed(rightSpeed);
        motors.setM2Speed(leftSpeed);
      }
      break;

    case GO_TO_GOAL:
      if (puckInPossession()) {
        Serial.println("🏃 Moving toward goal...");
        moveTowardGoalWithPControl(robotX, robotY);
      } else {
        Serial.println("❌ Lost puck — back to search.");
        currentState = SEARCH_PUCK;
      }
      break;

    case STOP:
      stopMovement();
      break;
  }
}

// ======= Pixy Check =======
bool pixySeesOrange() {
  pixy.ccc.getBlocks();
  return (pixy.ccc.numBlocks > 0 && pixy.ccc.blocks[0].m_signature == 1);
}

// ======= Angle Strike =======
void angleStrike() {
  Serial.println("🎯 Quick angle + strike...");
  motors.setM1Speed(150);
  motors.setM2Speed(-150);
  delay(120);
  motors.setM1Speed(300);
  motors.setM2Speed(300);
  delay(400);
  stopMovement();
  Serial.println("✅ Angled strike complete.");
}

// ======= Strike =======
void strikePuck() {
  pixy.ccc.getBlocks();
  if (pixy.ccc.numBlocks == 0 || pixy.ccc.blocks[0].m_signature != 1) {
    Serial.println("🔍 Searching for puck...");
    motors.setM1Speed(100);
    motors.setM2Speed(-100);
    return;
  }
  int x = pixy.ccc.blocks[0].m_x;
  int correction = (x - 160) / 2;
  int baseSpeed = 300;
  int leftSpeed = constrain(baseSpeed - correction, 75, 400);
  int rightSpeed = constrain(baseSpeed + correction, 75, 400);
  motors.setM1Speed(rightSpeed);
  motors.setM2Speed(leftSpeed);
  delay(400);
  stopMovement();
  Serial.println("💥 Puck struck – move complete.");
}

// ======= Search Puck =======
void searchPuck() {
  pixy.ccc.getBlocks();
  if (pixy.ccc.numBlocks > 0 && pixy.ccc.blocks[0].m_signature == 1) {
    motors.setM1Speed(0);
    motors.setM2Speed(0);   
    Serial.println("🧡 Puck detected.");
  } else {
    motors.setM1Speed(100);
    motors.setM2Speed(-100);
    Serial.println("🔍 Searching for puck...");
  }
}

// ======= Go to the Puck (Proportional) =======
void Go_TO_THE_PUCK() {
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks == 0 || pixy.ccc.blocks[0].m_signature != 1) {
    stopMovement();
    Serial.println("❌ Puck lost.");
    return;
  }

  // if (puckInPossession()) {
  //   stopMovement();
  //   Serial.println("✅ Puck acquired! Aligning to goal...");
  //   currentState = ALIGN_ROBOT;
  //   return;
  // }
  int x = pixy.ccc.blocks[0].m_x;
  int y = pixy.ccc.blocks[0].m_y;



  //int baseSpeed = (y < 100) ? 400 : 200;
  int xCenter = 160;          // Center of Pixy frame
  int xError = x - xCenter;   // -ve = puck left, +ve = puck right
  int correction = xError / 2; 

  int leftSpeed = constrain(baseSpeed - correction, 75, 400);
  int rightSpeed = constrain(baseSpeed + correction, 75, 400);

  motors.setM1Speed(rightSpeed);
  motors.setM2Speed(leftSpeed);

  Serial.print("🧡 Puck X: ");
  Serial.print(x);
  Serial.print(" | Y: ");
  Serial.print(y);
  Serial.print(" | BaseSpeed: ");
  Serial.print(baseSpeed);
  Serial.print(" | Correction: ");
  Serial.println(correction);
  Serial.println("Ping");
  Serial.print(getPingDistance());

  if (puckInPossession()) {
    stopMovement();
    Serial.println("✅ Puck acquired! Aligning to goal...");
    currentState = ALIGN_ROBOT;
    return;
  }
}

// ======= Go to Goal Movement =======
void moveTowardGoalWithPControl(float robotX, float robotY) {
  float goalAngleRad = atan2(goalY - robotY, goalX - robotX);
  float goalAngleDeg = goalAngleRad * 180 / PI;
  float currentAngle = getCurrentAngle();
  float error = angleError(goalAngleDeg, currentAngle);
  float correction = Kp * error;

  int baseSpeed = 300;
  int leftSpeed = constrain(baseSpeed - correction, 75, 400);
  int rightSpeed = constrain(baseSpeed + correction, 75, 400);

  motors.setM1Speed(rightSpeed);
  motors.setM2Speed(leftSpeed);

  Serial.print("🎯 Goal Heading: ");
  Serial.print(goalAngleDeg);
  Serial.print(" | Current: ");
  Serial.print(currentAngle);
  Serial.print(" | Error: ");
  Serial.println(error);
}

// ======= Puck Possession Check =======
bool puckInPossession() {
  long dist = getPingDistance();
  return (dist > 2 && dist < 6);
}

// ======= IMU Functions =======
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

// ======= Ping Sensor =======
long getPingDistance() {
  long duration, cm;
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);
  cm = duration / 29 / 2;
  return cm;
}

// ======= Stop All Motors =======
void stopMovement() {
  motors.setM1Speed(0);
  motors.setM2Speed(0);
}

void updateRobotPositionFromZigbee() {
  static unsigned long previousMillis = 0;
  const long interval = 500;

  if (millis() - previousMillis >= interval) {
    previousMillis = millis();
    Serial1.print('?');  // Request data from ZigBee
    Serial.println("Sent data request");
  }

  if (Serial1.available()) {
    String receivedData = Serial1.readStringUntil('\n');
    receivedData.trim();

    Serial.print("Received: ");
    Serial.println(receivedData);

    int firstComma = receivedData.indexOf(',');
    int secondComma = receivedData.indexOf(',', firstComma + 1);
    int thirdComma = receivedData.indexOf(',', secondComma + 1);

    if (firstComma != -1 && secondComma != -1 && thirdComma != -1) {
      String matchByte = receivedData.substring(0, firstComma);
      String xCoordStr = receivedData.substring(firstComma + 1, secondComma);
      String yCoordStr = receivedData.substring(secondComma + 1, thirdComma);

      robotX = xCoordStr.toFloat();
      robotY = yCoordStr.toFloat();

      Serial.print("Updated Robot Position: X=");
      Serial.print(robotX);
      Serial.print(" Y=");
      Serial.println(robotY);
    }
   }
}
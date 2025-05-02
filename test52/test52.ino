// ======= Includes =======
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <DualMAX14870MotorShield.h>
#include <Pixy2.h>
#include <Encoder.h>

// ======= Pins & Constants =======
const int pingPin = 3;
const float goalX = 56.0;
const float goalY = 211.0;
float robotX = 0.0;
float robotY = 0.0;
int n = 1;
int baseSpeed = 200;
const int xCenter = 160; // Center of Pixy frame

float targetHeading = 0.0;

// ======= Hardware Setup =======
DualMAX14870MotorShield motors;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Pixy2 pixy;
Encoder leftEncoder(2, 3);    // Global scope for use in all functions
Encoder rightEncoder(18, 19); // Global scope

// ======= FSM States =======
enum RobotState {
  SEARCH_PUCK,
  GO_TO_THE_PUCK,
  ALIGN_ROBOT,
  GO_TO_GOAL,
  STOP,
  DEFENCE
};
RobotState currentState;

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
  currentState = SEARCH_PUCK;
}

void loop() {
  switch (currentState) {
    case SEARCH_PUCK: {
      searchPuck();
      if (pixySeesOrange()) {
        currentState = GO_TO_THE_PUCK;
      }
      break;
    }

    case GO_TO_THE_PUCK: {
      if (pixySeesOrange()) {
        goToThePuck();
      } else {
        currentState = SEARCH_PUCK;
      }
      break;
    }

    // Additional states can go here (ALIGN_ROBOT, GO_TO_GOAL, etc.)
  }
}

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

void goToThePuck() {
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks) {
    int x = pixy.ccc.blocks[0].m_x;
    int y = pixy.ccc.blocks[0].m_y;

    int xError = x - xCenter;
    int visionCorrection = xError / 2;

    long leftCount = leftEncoder.read();
    long rightCount = rightEncoder.read();
    long encoderError = rightCount - leftCount;
    int encoderCorrection = encoderError / 5;

    int leftSpeed = constrain(baseSpeed - visionCorrection + encoderCorrection, 75, 400);
    int rightSpeed = constrain(baseSpeed + visionCorrection - encoderCorrection, 75, 400);

    motors.setM1Speed(rightSpeed); // Right motor
    motors.setM2Speed(leftSpeed);  // Left motor

    Serial.print("🧡 Puck X: ");
    Serial.print(x);
    Serial.print(" | Y: ");
    Serial.print(y);
    Serial.print(" | Vision Correction: ");
    Serial.print(visionCorrection);
    Serial.print(" | Encoder Correction: ");
    Serial.println(encoderCorrection);
    Serial.print("Left Encoder: ");
    Serial.print(leftCount);
    Serial.print(" | Right Encoder: ");
    Serial.println(rightCount);
    Serial.println("Ping Distance: ");
    
  } else {
    motors.setM1Speed(0);
    motors.setM2Speed(0);
  }
}

bool pixySeesOrange() {
  pixy.ccc.getBlocks();
  return (pixy.ccc.numBlocks > 0 && pixy.ccc.blocks[0].m_signature == 1);
}

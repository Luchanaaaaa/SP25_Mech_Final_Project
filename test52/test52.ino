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
int baseSpeed = 150;
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
  //currentState = (n == 0) ? SET_ANGLE : SEARCH_PUCK;
  currentState= SEARCH_PUCK;
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
      if (pixySeesOrange() && !puckInPossession()) {
        goToThePuck();
      } else if (puckInPossession()) {
        currentState = ALIGN_ROBOT;
      } else {
        currentState = SEARCH_PUCK;
      }
      break;
    }

    case ALIGN_ROBOT: {
      if (puckInPossession()){

      alignToGoal();  // You should define this function
      }else{
        currentState = SEARCH_PUCK;

      }
     // currentState = GO_TO_GOAL;
      break;
    }

    case GO_TO_GOAL: {
      if (puckInPossession()){
      goToGoal(); // You should define this function
     break;
      }else{
        currentState = SEARCH_PUCK;
      }
    }

     case STOP: {
    motors.setM1Speed(0);
    motors.setM2Speed(0);
     break;
     }
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

bool puckInPossession() {
  long total = 0;
  const int samples = 5;

  for (int i = 0; i < samples; i++) {
    total += getPingDistance();
    delay(5);  // Small delay for stability
  }

  long avg = total / samples;

  Serial.print("📏 Avg Ping: ");
  Serial.println(avg);

  return (avg > 2 && avg < 6);  // Puck possession zone
}

void alignToGoal() {
  // Check for green goal (signature 2)
  pixy.ccc.getBlocks();
  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    if (pixy.ccc.blocks[i].m_signature == 2) {
      motors.setM1Speed(0);
      motors.setM2Speed(0);
      Serial.println("✅ Green goal detected – aligned!");
      currentState = GO_TO_GOAL;
      return;
    }
  }

  // Get current heading from IMU
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float heading = euler.x(); // yaw angle

  if (heading < 0) heading += 360;

  Serial.print("🔄 Aligning... Heading: ");
  Serial.println(heading);

  // Rotate based on heading zones
  if (heading >= 270 && heading <= 360) {
    // Turn clockwise
    motors.setM1Speed(100);
    motors.setM2Speed(-100);
    Serial.println("↻ Turning clockwise to align.");
  }
  else if (heading >= 0 && heading <= 90) {
    // Turn counter-clockwise
    motors.setM1Speed(-100);
    motors.setM2Speed(100);
    Serial.println("↺ Turning counter-clockwise to align.");
  }
  else {
    // In mid-range (90–270°), choose shortest direction
    // Optional: could stop or apply smarter logic here
    motors.setM1Speed(80);
    motors.setM2Speed(-80);
    Serial.println("🔁 Mid range – rotating to search.");
  }
}



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

void goToGoal() {
  pixy.ccc.getBlocks();

  // Check if green goal is visible
  if (pixy.ccc.numBlocks > 0) {
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      if (pixy.ccc.blocks[i].m_signature == 2) {
        int x = pixy.ccc.blocks[i].m_x;
        int y = pixy.ccc.blocks[i].m_y;

        int xError = x - xCenter;  // -ve = goal is left, +ve = right
        int correction = xError / 2;

        int leftSpeed = constrain(baseSpeed - correction, 75, 255);
        int rightSpeed = constrain(baseSpeed + correction, 75, 255);

        motors.setM1Speed(rightSpeed); // Right motor
        motors.setM2Speed(leftSpeed);  // Left motor

        Serial.print("🎯 Green goal X: ");
        Serial.print(x);
        Serial.print(" | Y: ");
        Serial.print(y);
        Serial.print(" | X Error: ");
        Serial.print(xError);
        Serial.print(" | Correction: ");
        Serial.println(correction);

        // // Optional: Stop if goal is very close (based on Y or ping)
        // if (y > 180 || getPingDistance() < 10) {
        //   motors.setM1Speed(0);
        //   motors.setM2Speed(0);
        //   Serial.println("✅ Reached goal.");
        //   currentState = STOP;
        // }

        return; // Found the green, acted on it
      }
    }
  }

  // If no green detected
  Serial.println("❌ Green goal not found. Searching...");
  motors.setM1Speed(100);
  motors.setM2Speed(-100); // Rotate to search
}






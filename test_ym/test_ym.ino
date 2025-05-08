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
int p = 0;
const float goalX = 57.0;
const float goalY = 6.0;
float robotX = 0.0;
float robotY = 0.0;
int n = 1;
int baseSpeed = 150;
const int xCenter = 160; // Center of Pixy frame
float yawOffset = 0;
float backtrackTargetYaw = 180;   // assume 180 right now
const int WALL_DISTANCE_THRESHOLD = 50;
float targetHeading = 0.0;

///////  PID Control Parameters
float driveKp = 2.5;
float driveKi = 0.15;
float driveKd = 0.8;
float driveIntegral = 0.0;
float driveLastError = 0.0;
unsigned long driveLastTime = 0;

// turning pid
float turnKp = 3.5;  // Higher proportional coefficient for turning
float turnKi = 0.15;
float turnKd = 1.5;  // Higher derivative coefficient for turning, reduces oscillation
float turnIntegral = 0.0;
float turnLastError = 0.0;
unsigned long turnLastTime = 0;

// ping
#define PING_WINDOW_SIZE 5
long pingReadings[PING_WINDOW_SIZE];
int pingIndex = 0;

// ======= Hardware Setup =======
DualMAX14870MotorShield motors;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Pixy2 pixy;
Encoder leftEncoder(2, 3);    // Global scope for use in all functions
Encoder rightEncoder(18, 19); // Global scope

// Control
float turnTargetAngle = 0.0;


// ======= FSM States =======
enum RobotState {
  SEARCH_PUCK,
  CHECK_PUCK_ALIGNMENT,
  GET_BEHIND_PUCK,
  TURN_TO_ANGLE,
  GO_TO_THE_PUCK,
  ALIGN_TO_GOAL,
};
RobotState currentState;

// SubStates for driving toward an angle
enum DriveSubState {
  TURNING_TO_TARGET,   // Turning phase
  DRIVING_STRAIGHT     // Driving straight phase
};
DriveSubState currentDriveSubState = TURNING_TO_TARGET;

// Motor balance factor - used to compensate for differences between left and right motors
float motorBalanceFactor = 1.0;  // >1 means left wheel faster than right, <1 means right wheel faster than left

// === Function Prototypes ===
float getNormalizedYaw();
void driveTowardYaw(float targetYaw, int baseSpeed = 150);
bool turnWithPID(float targetAngle, int maxTurnSpeed = 150);
void goToThePuck(int customSpeed = 150);

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

  // set up the imu sensor
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  yawOffset = euler.x();
  
  // Initialize PID time variables
  driveLastTime = millis();
  turnLastTime = millis();
  
  // Reset integral terms
  driveIntegral = 0.0;
  turnIntegral = 0.0;

  motorBalanceFactor = 1.0;
}

void loop() {
  Serial.print("distance");
  Serial.println(getSmoothedPingDistance());
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float rawYaw = euler.x(); 
  float yaw = rawYaw - yawOffset;
  if (yaw < 0) yaw += 360;
  if (yaw >= 360) yaw -= 360;

  Serial.print("Yaw (Y): ");
  Serial.println(yaw);

  switch (currentState) {
    case SEARCH_PUCK: {
      searchPuck();
      if (pixySeesOrange()) {
        currentState = CHECK_PUCK_ALIGNMENT;
      }
      break;
    }

    case CHECK_PUCK_ALIGNMENT: {
      if (pixySeesOrange()) {
        delay(500);
        pixy.ccc.getBlocks();
        int puckX = pixy.ccc.blocks[0].m_x;
        if (yaw > 270 || yaw < 90) {
          Serial.println("Angle aligned");
          currentState = GO_TO_THE_PUCK;
        } else {
          Serial.println("❌ Angle not aligned");
          // Reset PID variables before switching to GET_BEHIND_PUCK state
          driveIntegral = 0.0;
          driveLastError = 0.0;
          driveLastTime = millis();
          currentState = GET_BEHIND_PUCK;
        }
      } else {
        currentState = SEARCH_PUCK;
      }
      break;
    }
    case GET_BEHIND_PUCK: {
      int distance = getSmoothedPingDistance();
      Serial.println("GET_BEHIND_PUCK⤴");
      
      // Set a larger warning distance to start slowing down earlier
      const int SLOW_DOWN_THRESHOLD = 30;  // Larger than WALL_DISTANCE_THRESHOLD
      
      if (distance > SLOW_DOWN_THRESHOLD || pixySeesOrange()) {
        // Normal speed when far from wall
        driveTowardYaw(180, 150);
        Serial.println("🚗 Driving Toward Target Yaw!");
      } 
      else if (distance > WALL_DISTANCE_THRESHOLD) {
        // In deceleration zone, speed proportional to distance
        int reducedSpeed = map(distance, WALL_DISTANCE_THRESHOLD, SLOW_DOWN_THRESHOLD, 80, 150);
        driveTowardYaw(180, reducedSpeed);
        Serial.print("⚠️ Approaching wall, reducing speed: ");
        Serial.println(reducedSpeed);
      }
      else {
        // Stop briefly to ensure complete stop
        motors.setM1Speed(0);
        motors.setM2Speed(0);
        delay(100);  // Brief stop
        
        // Reset PID variables before switching to TURN_TO_ANGLE state
        turnIntegral = 0.0;
        turnLastError = 0.0;
        turnLastTime = millis();
        
        // Switch to turning state
        currentState = TURN_TO_ANGLE;
      }
      break;
    }
    case TURN_TO_ANGLE:{
      Serial.println("Executing turn...");
      turnTargetAngle = 0.0;  // Set target angle to 0 degrees
      
      // Call turnWithPID function for PID-controlled turning
      // If function returns true, turning is complete
      if (turnWithPID(turnTargetAngle)) {
        Serial.println("✅ Turn complete");
        currentState = SEARCH_PUCK;  // Switch to search state after turning
      }
      // If function returns false, will continue turning in next loop() cycle
      break;
    }
    case GO_TO_THE_PUCK: {
      int distance = readPingDistance();
      Serial.print("Distance to puck: ");
      Serial.println(distance);
      
      // Check if puck has been contacted
      if (distance <= 3 || distance > 300) {  // Very close, indicating contact with puck
        Serial.println("✅ Puck contacted, preparing to align with goal");
        motors.setM1Speed(0);
        motors.setM2Speed(0);
        delay(200);  // Ensure complete stop
        currentState = ALIGN_TO_GOAL;
        break;
      }
      
      goToThePuck(baseSpeed);
      Serial.println("GO_TO_THE_PUCK");
      break;
    }
    case ALIGN_TO_GOAL:{
      int p=1;
      decideTurnDirection();
      updateRobotPositionFromZigbee();

      break;
    }
  }
  
  
  delay(50);
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

void goToThePuck(int customSpeed) {  
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

    int leftSpeed = constrain(customSpeed - visionCorrection + encoderCorrection, 75, 400);
    int rightSpeed = constrain(customSpeed + visionCorrection - encoderCorrection, 75, 400);

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
    Serial.print("Speed: ");
    Serial.println(customSpeed);
    Serial.println("Ping Distance: ");
    Serial.println(getSmoothedPingDistance());
  }
}

bool pixySeesOrange() {
  pixy.ccc.getBlocks();
  return (pixy.ccc.numBlocks > 0 && pixy.ccc.blocks[0].m_signature == 1);
}


/////////////////////Ping//////////////////////////////////
long getSmoothedPingDistance() {
  long newReading = readPingDistance();
  if (newReading < 2 || newReading > 500) return -1; // filter out junk

  pingReadings[pingIndex] = newReading;
  pingIndex = (pingIndex + 1) % PING_WINDOW_SIZE;

  long sum = 0;
  int validCount = 0;
  for (int i = 0; i < PING_WINDOW_SIZE; i++) {
    if (pingReadings[i] > 0) {
      sum += pingReadings[i];
      validCount++;
    }
  }

  if (validCount == 0) return -1;
  return sum / validCount;
}


long readPingDistance() {
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  pinMode(pingPin, INPUT);
  long duration = pulseIn(pingPin, HIGH);

  long distanceCm = duration * 0.034 / 2;
  return distanceCm;
}


// IMU
void driveTowardYaw(float targetYaw, int baseSpeed) {
  static float lastTargetYaw = -999;  // Initialize to an impossible angle value
  
  // Reset substate if target angle changes
  if (targetYaw != lastTargetYaw) {
    currentDriveSubState = TURNING_TO_TARGET;
    lastTargetYaw = targetYaw;
    
    // Reset PID variables
    turnIntegral = 0.0;
    turnLastError = 0.0;
    turnLastTime = millis();
    
    driveIntegral = 0.0;
    driveLastError = 0.0;
    driveLastTime = millis();
    
    Serial.print("⚙️ New target angle: ");
    Serial.println(targetYaw);
  }
  
  // Get current angle
  float yaw = getNormalizedYaw();
  
  // Execute action for current substate
  if (currentDriveSubState == TURNING_TO_TARGET) {
    // Execute turning
    if (turnWithPID(targetYaw, 120)) {  // Use lower max turning speed
      // Turning complete, switch to driving straight
      Serial.println("✅ Turn complete, starting straight drive");
      currentDriveSubState = DRIVING_STRAIGHT;
      
      // Reset driving PID variables
      driveIntegral = 0.0;
      driveLastError = 0.0;
      driveLastTime = millis();
      
      // Brief pause to ensure stability
      motors.setM1Speed(0);
      motors.setM2Speed(0);
      delay(200);
    }
  } else {
    // Execute straight driving
    // Calculate error
    float yawError = targetYaw - yaw;
    if (yawError > 180) yawError -= 360;
    if (yawError < -180) yawError += 360;
    
    // Check if need to return to turning state (if deviation too large)
    if (abs(yawError) > 25.0) {
      Serial.println("⚠️ Deviation too large, returning to turning state");
      currentDriveSubState = TURNING_TO_TARGET;
      
      // Stop motors
      motors.setM1Speed(0);
      motors.setM2Speed(0);
      delay(100);
      
      // Reset turning PID variables
      turnIntegral = 0.0;
      turnLastError = 0.0;
      turnLastTime = millis();
      return;
    }
    
    // Calculate time difference
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - driveLastTime) / 1000.0; // Convert to seconds
    
    // Safety check
    if (deltaTime <= 0 || deltaTime > 0.5) {
      deltaTime = 0.01; // Use safe default if time is abnormal
    }
    
    driveLastTime = currentTime;
    
    // Integral term
    driveIntegral += yawError * deltaTime;
    driveIntegral = constrain(driveIntegral, -40, 40); // Prevent integral saturation
    
    // Derivative term
    float derivative = (yawError - driveLastError) / deltaTime;
    driveLastError = yawError;
    
    // PID calculation - more aggressive correction
    float correction = driveKp * yawError + driveKi * driveIntegral + driveKd * derivative;
  
    // Adjust control strength based on error magnitude
    if (abs(yawError) < 5) {
      // Lighter correction for small errors
      correction = (driveKp * 0.6) * yawError + (driveKi * 0.8) * driveIntegral + (driveKd * 1.2) * derivative;
    } else if (abs(yawError) > 15) {
      // Stronger correction for large errors
      correction = (driveKp * 1.3) * yawError + driveKi * driveIntegral + driveKd * derivative;
    }
  
    // Apply motor balance factor
    float leftAdjustment = 0;
    float rightAdjustment = 0;
    
    if (motorBalanceFactor > 1.0) {
      // Left wheel faster than right
      leftAdjustment = baseSpeed * (motorBalanceFactor - 1.0);
    } else if (motorBalanceFactor < 1.0) {
      // Right wheel faster than left
      rightAdjustment = baseSpeed * (1.0 - motorBalanceFactor);
    }
  
    // Set motor speeds
    int leftSpeed = constrain(baseSpeed - correction + leftAdjustment, 70, 200);
    int rightSpeed = constrain(baseSpeed + correction + rightAdjustment, 70, 200);
  
    motors.setM1Speed(rightSpeed);
    motors.setM2Speed(leftSpeed);
  
    // Output debug info
    Serial.print("🚗 Driving: Current=");
    Serial.print(yaw);
    Serial.print("° Target=");
    Serial.print(targetYaw);
    Serial.print("° Error=");
    Serial.print(yawError);
    Serial.print("° P=");
    Serial.print(driveKp * yawError);
    Serial.print(" I=");
    Serial.print(driveKi * driveIntegral);
    Serial.print(" D=");
    Serial.print(driveKd * derivative);
    Serial.print(" Left=");
    Serial.print(leftSpeed);
    Serial.print(" Right=");
    Serial.println(rightSpeed);
  }
}

bool turnWithPID(float targetAngle, int maxTurnSpeed) {
  const int minSpeed = 100;  // Adjust this value until it can move even with small errors
  const int maxSpeed = 150;

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float yaw = euler.x(); 
  yaw = yaw - yawOffset;
  if (yaw < 0) yaw += 360;
  if (yaw >= 360) yaw -= 360;
  
  // Calculate error (shortest path)
  float yawError = targetAngle - yaw;
  if (yawError > 180) yawError -= 360;
  if (yawError < -180) yawError += 360;
  
  // Restore smaller error threshold for increased precision
  if (abs(yawError) < 2.0) {
    motors.setM1Speed(0);
    motors.setM2Speed(0);
    return true; // Turning complete
  }
  
  // Calculate time difference - using turn-specific time variable
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - turnLastTime) / 1000.0; // Convert to seconds
  
  // Add time safety check
  if (deltaTime <= 0 || deltaTime > 0.5) {
    deltaTime = 0.01; // Use safe default if time is abnormal
  }
  
  turnLastTime = currentTime;
  
  // Integral term - using turn-specific integral variable
  turnIntegral += yawError * deltaTime;
  turnIntegral = constrain(turnIntegral, -30, 30); // Limit integral term to prevent saturation
  
  // Derivative term - using turn-specific error variable
  float derivative = (yawError - turnLastError) / deltaTime;
  turnLastError = yawError;
  
  // Calculate PID output - using turn-specific PID parameters
  float pidOutput = turnKp * yawError + turnKi * turnIntegral + turnKd * derivative;
  
  // Dynamically adjust PID parameters based on error magnitude
  if (abs(yawError) < 10) {
    // Near target, reduce P gain, increase D gain to reduce oscillation
    pidOutput = (turnKp * 0.7) * yawError + turnKi * turnIntegral + (turnKd * 1.5) * derivative;
  }
  
  int turnSpeed = constrain(abs(pidOutput), minSpeed, maxSpeed);

  if (abs(yawError) < 5.0 && abs(pidOutput) < 5.0) {
    // Target reached, stop motors
    return true;
  }
  
  // Set turning direction
  if (yawError > 0) { // Need clockwise turn
    motors.setM1Speed(turnSpeed);    // Right motor forward
    motors.setM2Speed(-turnSpeed);   // Left motor reverse
  } else { // Need counterclockwise turn
    motors.setM1Speed(-turnSpeed);   // Right motor reverse
    motors.setM2Speed(turnSpeed);    // Left motor forward
  }
  
  // Output debug information
  Serial.print("🔄 Turning: Current=");
  Serial.print(yaw);
  Serial.print("° Target=");
  Serial.print(targetAngle);
  Serial.print("° Error=");
  Serial.print(yawError);
  Serial.print("° Speed=");
  Serial.print(turnSpeed);
  Serial.print(" P=");
  Serial.print(turnKp * yawError);
  Serial.print(" I=");
  Serial.print(turnKi * turnIntegral);
  Serial.print(" D=");
  Serial.println(turnKd * derivative);
  
  return false; // Turning not complete
}

void decideTurnDirection() {
  float dx = goalX - robotX;
  float dy = goalY - robotY;
  float theta = atan2(dy, dx) * 180.0 / PI;
  float IMUtarget = (dx < 0) ? theta + 180 : 360 - abs(theta);
  if (IMUtarget < 0) IMUtarget += 360;
  if (IMUtarget >= 360) IMUtarget -= 360;

  float currentYaw = IMUAngle();
  float diff = IMUtarget - currentYaw;
  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;
  diff = abs(diff);

  long leftCount = leftEncoder.read();
  long rightCount = rightEncoder.read();
  long encoderError = rightCount - leftCount;
  int encoderCorrection = encoderError / 5;

  int leftSpeed, rightSpeed;

  if (dx < 0) {
    if ((currentYaw >= 270 && currentYaw <= 360) || (currentYaw >= 0 && currentYaw <= IMUtarget)) {
      leftSpeed = constrain(baseSpeed + diff + encoderCorrection, 75, 400);
      rightSpeed = constrain(baseSpeed - diff - encoderCorrection, 75, 400);
    } else {
      leftSpeed = constrain(baseSpeed - diff + encoderCorrection, 75, 400);
      rightSpeed = constrain(baseSpeed + diff - encoderCorrection, 75, 400);
    }
  } else {
    if ((currentYaw >= 0 && currentYaw <= 90) || (currentYaw >= IMUtarget && currentYaw <= 360)) {
      leftSpeed = constrain(baseSpeed + diff + encoderCorrection, 75, 400);
      rightSpeed = constrain(baseSpeed - diff - encoderCorrection, 75, 400);
    } else {
      leftSpeed = constrain(baseSpeed - diff + encoderCorrection, 75, 400);
      rightSpeed = constrain(baseSpeed + diff - encoderCorrection, 75, 400);
    }
  }

  motors.setM1Speed(rightSpeed);
  motors.setM2Speed(leftSpeed + 50);
}

void updateRobotPositionFromZigbee() {
  static unsigned long previousMillis = 0;
  const long interval = 500;

  if (millis() - previousMillis >= interval) {
    previousMillis = millis();
    Serial1.print('?');
    Serial.println("Sent data request");
  }

  if (Serial1.available()) {
    String receivedData = Serial1.readStringUntil('\n');
    receivedData.trim();

    int firstComma = receivedData.indexOf(',');
    int secondComma = receivedData.indexOf(',', firstComma + 1);
    int thirdComma = receivedData.indexOf(',', secondComma + 1);

    if (firstComma != -1 && secondComma != -1 && thirdComma != -1) {
      String xCoordStr = receivedData.substring(firstComma + 1, secondComma);
      String yCoordStr = receivedData.substring(secondComma + 1, thirdComma);

      robotX = xCoordStr.toFloat();
      robotY = yCoordStr.toFloat();

      Serial1.print("Updated Robot Position: X=");
      Serial1.print(robotX);
      Serial1.print(" Y=");
      Serial1.println(robotY);
    }
  } else {
    Serial1.print("Done");
  }
}

double IMUAngle(){
  sensors_event_t event;
  bno.getEvent(&event);
  return event.orientation.x;
}

// Get normalized yaw angle
float getNormalizedYaw() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float yaw = euler.x() - yawOffset;
  if (yaw < 0) yaw += 360;
  if (yaw >= 360) yaw -= 360;
  return yaw;
}
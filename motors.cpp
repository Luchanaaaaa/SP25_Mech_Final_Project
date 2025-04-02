#include "motors.h"

// Global volatile encoder counters
static volatile long encoderCountLeft  = 0;
static volatile long encoderCountRight = 0;

// Forward declarations for ISR functions
void handleLeftEncoder();
void handleRightEncoder();

// Constants for feedback loops and turning
static const int TURN_COUNTS                 = 140;      // How many encoder counts to turn (adjust as needed)
static const unsigned long FEEDBACK_INTERVAL = 5; // ms between feedback updates
static const unsigned long MOVE_DURATION     = 5; // ms to run forward (or curve) feedback loop (adjust or replace with your own exit condition)
static const int KP = 5;                           // Proportional gain for feedback (tune this value)

Motor::Motor(){ }

void Motor::begin() {
  // Setup encoder pins
  pinMode(20, INPUT);  // Left encoder
  pinMode(21, INPUT);  // Right encoder

  // Attach interrupts to encoder pins
  attachInterrupt(digitalPinToInterrupt(21), handleLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(20), handleRightEncoder, RISING);
}

// Change motor speeds from current values to targets.
void Motor::setMotors(int targetM1, int targetM2) {
  motors.setM1Speed(targetM1);
  motors.setM2Speed(targetM2);
}

//-----------------------------------------------------
// MOVE FORWARD WITH FEEDBACK
//-----------------------------------------------------
void Motor::moveForward() {
  long prevLeft = encoderCountLeft;
  long prevRight = encoderCountRight;
  int leftSpeed = SPEED + 2;
  int rightSpeed = SPEED;

  // Ramp up to starting speeds
  setMotors(leftSpeed, rightSpeed);

  unsigned long lastUpdate = millis();
  unsigned long startTime  = millis();
  
  while (millis() - startTime < MOVE_DURATION) {
    if (millis() - lastUpdate >= FEEDBACK_INTERVAL) {
      long currentLeft  = encoderCountLeft;
      long currentRight = encoderCountRight;
      long deltaLeft  = currentLeft  - prevLeft;
      long deltaRight = currentRight - prevRight;
      
      // Compute error between wheels; if left is moving too fast, error > 0, and vice versa.
      int error = deltaLeft - deltaRight + 40;
      int correction = KP * error;
      
      // Adjust speeds to compensate for the difference.
      leftSpeed  = SPEED - correction;
      rightSpeed = SPEED + correction;
      
      // Keep speeds within 75 to 400.
      leftSpeed  = constrain(leftSpeed,  75, 400);
      rightSpeed = constrain(rightSpeed, 75, 400);
      
      motors.setM1Speed(leftSpeed + 40);
      motors.setM2Speed(rightSpeed);
      
      // Update for next interval
      prevLeft = currentLeft;
      prevRight = currentRight;
      lastUpdate = millis();
    }
    delay(1); // short delay to avoid a tight busy loop
  }
}

//-----------------------------------------------------
// TURN LEFT: MOVE ONLY THE RIGHT WHEEL
//-----------------------------------------------------
void Motor::turnLeft() {
  long startCount = encoderCountRight;
  setMotors(0, SPEED + 40);
  while ((encoderCountRight - startCount) < TURN_COUNTS - 3) {
    delay(1);
  }
  stop();
}

//-----------------------------------------------------
// TURN RIGHT: MOVE ONLY THE LEFT WHEEL
//-----------------------------------------------------
void Motor::turnRight() {
  long startCount = encoderCountLeft;
  setMotors(SPEED + 40, 0);
  while ((encoderCountLeft - startCount) < TURN_COUNTS - 7) {
    delay(1);
  }
  stop();
}

//-----------------------------------------------------
// TURN AROUND (180° turn)
//-----------------------------------------------------
//
// This implementation simply spins the robot in place
// using both wheels. (You could add a feedback loop here as well.)
void Motor::turnAround() {
  long startCount = encoderCountLeft;
  setMotors(-SPEED, SPEED + 18);
  while ((encoderCountLeft - startCount) < 146) {
    delay(1);
  }
  stop();
}

//-----------------------------------------------------
// MOVE FORWARD WITH A SLIGHT LEFT CURVE
//-----------------------------------------------------
void Motor::moveForwardLeft() {
  const int OFFSET = 20;  // how much slower the left wheel should be
  int baseLeft  = SPEED - OFFSET;
  int baseRight = SPEED;
  
  long prevLeft  = encoderCountLeft;
  long prevRight = encoderCountRight;
  
  setMotors(baseLeft, baseRight);
  unsigned long lastUpdate = millis();
  unsigned long startTime  = millis();
  
  // For a left curve, we expect the left wheel to register fewer counts
  int desiredError = -5;
  
  while (millis() - startTime < MOVE_DURATION) {
    if (millis() - lastUpdate >= FEEDBACK_INTERVAL) {
      long currentLeft  = encoderCountLeft;
      long currentRight = encoderCountRight;
      long deltaLeft  = currentLeft  - prevLeft;
      long deltaRight = currentRight - prevRight;
      
      // The measured error minus the desired error is used for correction.
      int error = (deltaLeft - deltaRight) - desiredError;
      int correction = KP * error;
      
      int leftSpeed  = baseLeft  - correction;
      int rightSpeed = baseRight + correction;
      
      leftSpeed  = constrain(leftSpeed,  75, 400);
      rightSpeed = constrain(rightSpeed, 75, 400);
      
      motors.setM1Speed(leftSpeed);
      motors.setM2Speed(rightSpeed);
      
      prevLeft  = currentLeft;
      prevRight = currentRight;
      lastUpdate = millis();
    }
    delay(1);
  }
  
}

//-----------------------------------------------------
// MOVE FORWARD WITH A SLIGHT RIGHT CURVE
//-----------------------------------------------------
void Motor::moveForwardRight() {
  const int OFFSET = 40;  // how much slower the right wheel should be
  int baseLeft  = SPEED;
  int baseRight = SPEED - OFFSET;
  
  long prevLeft  = encoderCountLeft;
  long prevRight = encoderCountRight;
  
  setMotors(baseLeft, baseRight);
  unsigned long lastUpdate = millis();
  unsigned long startTime  = millis();
  
  // For a right curve, we expect the left wheel to register more counts.
  int desiredError = 40;
  
  while (millis() - startTime < MOVE_DURATION) {
    if (millis() - lastUpdate >= FEEDBACK_INTERVAL) {
      long currentLeft  = encoderCountLeft;
      long currentRight = encoderCountRight;
      long deltaLeft  = currentLeft  - prevLeft;
      long deltaRight = currentRight - prevRight;
      
      int error = (deltaLeft - deltaRight) - desiredError;
      int correction = KP * error;
      
      int leftSpeed  = baseLeft  - correction;
      int rightSpeed = baseRight + correction;
      
      leftSpeed  = constrain(leftSpeed,  75, 400);
      rightSpeed = constrain(rightSpeed, 75, 400);
      
      motors.setM1Speed(leftSpeed + 40);
      motors.setM2Speed(rightSpeed);
      
      prevLeft  = currentLeft;
      prevRight = currentRight;
      lastUpdate = millis();
    }
    delay(1);
  }
}

//-----------------------------------------------------
// STOP THE MOTORS
//-----------------------------------------------------
void Motor::stop() {
  setMotors(0, 0);
}

//-----------------------------------------------------
// ENCODER INTERRUPT HANDLERS
//-----------------------------------------------------
void handleLeftEncoder() {
  encoderCountLeft++;
}

void handleRightEncoder() {
  encoderCountRight++;
}

//-----------------------------------------------------
// PRINT ENCODER COUNTS
//-----------------------------------------------------
void Motor::printEncoders() {
  Serial.print("Left Encoder: ");
  Serial.print(encoderCountLeft);
  Serial.print("   Right Encoder: ");
  Serial.println(encoderCountRight);
}

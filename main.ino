#include "ping.h"
#include "motors.h"
#include "pixy.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Define pins
#define PING_PIN 2

// Define distance threshold
#define DIST 26.5

// Define motor
Motor motor;

// Create a PingSensor object
PingSensor pingSense(PING_PIN);

// Create a PixySensor object
PixySensor pixy;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Define our possible states in the FSM
enum State {
  MOVE_FORWARD,
  MOVE_FORWARD_LEFT,
  MOVE_FORWARD_RIGHT,
  STOP_READ_PIXY,
  TURN_LEFT,
  TURN_RIGHT,
  TURN_AROUND,
  STOP
};



// The current state
State currentState = MOVE_FORWARD;

void setup() {
  //Serial.begin(9600);

  // Initialize devices
  motor.begin();
  leftIR.begin();
  rightIR.begin();
  pingSense.begin();
  pixy.begin();

  // Wait for a while for the BNO to start up
  if (!bno.begin())
  {
    Serial.print("No BNO055 detected");
    while (1);
  }
}




void loop() {
  // Read sensor values
  float distance      = pingSense.getDistanceCm();

  // Read angular velocity
  sensors_event_t gyroEvent;
  bno.getEvent(&gyroEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);

  Serial.print("Gyroscope (Angular Velocity) X: ");
  Serial.print(gyroEvent.gyro.x, 4);


  // Main FSM switch
  switch (currentState) {

    // -----------------------------------
    // MOVE_FORWARD
    // -----------------------------------
    case MOVE_FORWARD:
      //Serial.print("[STATE] FWD");
      //Serial.print("\t PING: ");
      //Serial.print(distance);
      //Serial.println(" cm");
      motor.moveForward();

      // Check if obstacle is close
      if (distance < DIST) 
      {
        currentState = STOP_READ_PIXY;
      }
      break;

    // -----------------------------------
    // MOVE_FORWARD_LEFT
    // -----------------------------------
    case MOVE_FORWARD_LEFT:
      motor.moveForwardLeft();

      // Possibly check if we need to switch to forward-right or normal forward
      if (distance < DIST) {
        currentState = STOP_READ_PIXY;
      }
      break;

    // -----------------------------------
    // MOVE_FORWARD_RIGHT
    // -----------------------------------
    case MOVE_FORWARD_RIGHT:
      //Serial.print("[STATE] FWD R");
      //Serial.print("\t PING: ");
      //Serial.print(distance);
      //Serial.println(" cm");
      motor.moveForwardRight();

      if (distance < DIST) {
        currentState = STOP_READ_PIXY;
      }
      break;

    // -----------------------------------
    // STOP_READ_PIXY
    // -----------------------------------
    case STOP_READ_PIXY: {
      //Serial.println("[STATE] READ PIXY");
      motor.stop();
      delay(500);

      // Read the signature from Pixy
      int signature = pixy.getSignature();
      Serial.print("[INFO] Pixy signature: ");
      Serial.println(signature);

      // For example:
      // 1 = green, 2 = red, 3 = blue
      if (signature == 1) {
        currentState = TURN_LEFT;
      }
      else if (signature == 2) {
        currentState = TURN_RIGHT;
      }
      else if (signature == 3) {
        currentState = TURN_AROUND;
      }
      else {
        // If unknown signature, just stop
        currentState = STOP;
      }
      break;
    }

    // -----------------------------------
    // TURN_LEFT
    // -----------------------------------
    case TURN_LEFT:
      //Serial.println("[STATE] L");
      motor.turnLeft();
      delay(500);
      currentState = MOVE_FORWARD;
      break;

    // -----------------------------------
    // TURN_RIGHT
    // -----------------------------------
    case TURN_RIGHT:
      //Serial.println("[STATE] R");
      motor.turnRight();
      delay(500);
      currentState = MOVE_FORWARD;
      break;

    // -----------------------------------
    // TURN_AROUND
    // -----------------------------------
    case TURN_AROUND:
      //Serial.println("[STATE] 180");
      motor.turnAround();
      delay(500);
      currentState = MOVE_FORWARD;
      break;

    // -----------------------------------
    // STOP
    // -----------------------------------
    case STOP:
      //Serial.println("[STATE] STOP");
      motor.stop();
      delay(5000);
      break;
  }
  delay(10);
}

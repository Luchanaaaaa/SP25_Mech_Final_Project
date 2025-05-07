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
const float goalX = 57.0;
const float goalY = 6.0;
float robotX = 0.0;
float robotY = 0.0;
int n = 1;
int baseSpeed = 200;
const int xCenter = 160; // Center of Pixy frame
float yawOffset = 0;
float backtrackTargetYaw = 180;   // assume 180 right now
const int WALL_DISTANCE_THRESHOLD = 50;
float targetHeading = 0.0;

// PID Control Parameters
float kP = 2.0;
float kI = 0.1;
float kD = 0.5;
float integral = 0.0;
float lastError = 0.0;
unsigned long lastTime = 0;


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
  GO_TO_GOAL,
};
RobotState currentState;

// === Function Prototypes ===
void driveTowardYaw(float targetYaw, int baseSpeed = 150);
bool turnWithPID(float targetAngle, int maxTurnSpeed = 150);

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

  delay(200);
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
        pixy.ccc.getBlocks();
        int puckX = pixy.ccc.blocks[0].m_x;

        if (yaw > 270 || yaw < 90) {
          Serial.println("角度对齐了");
          currentState = GO_TO_THE_PUCK;
        } else {
          Serial.println("❌角度没对齐");
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
      
      // 设置一个更大的预警距离，提前开始减速
      const int SLOW_DOWN_THRESHOLD = 40;  // 比WALL_DISTANCE_THRESHOLD大
      
      if (distance > SLOW_DOWN_THRESHOLD) {
        // 远离墙壁时正常速度
        driveTowardYaw(180, 150);
      } 
      else if (distance > WALL_DISTANCE_THRESHOLD) {
        // 进入减速区间，速度与距离成正比
        int reducedSpeed = map(distance, WALL_DISTANCE_THRESHOLD, SLOW_DOWN_THRESHOLD, 80, 150);
        driveTowardYaw(180, reducedSpeed);
        Serial.print("⚠️ 接近墙壁，减速: ");
        Serial.println(reducedSpeed);
      }
      else {
        // 先停止一小段时间，确保完全静止
        motors.setM1Speed(0);
        motors.setM2Speed(0);
        delay(100);  // 短暂停止
        
        // 然后切换到转向状态
        currentState = TURN_TO_ANGLE;
      }
      break;
    }
    case TURN_TO_ANGLE:{
      Serial.println("执行转向...");
      turnTargetAngle = 0.0;  // 设置目标角度为0度
      
      // 调用turnWithPID函数进行PID控制转向
      // 如果函数返回true，表示转向完成
      if (turnWithPID(turnTargetAngle)) {
        Serial.println("✅ 转向完成");
        currentState = SEARCH_PUCK;  // 转向完成后切换到搜索状态
      }
      // 如果函数返回false，会在下一次loop()中继续执行转向
      break;
    }

    case GO_TO_THE_PUCK: {
      // if (pixySeesOrange()) {
      //   goToThePuck();
      // } else {
      //   currentState = SEARCH_PUCK;
      // }
      goToThePuck();
      Serial.println("GO_TO_THE_PUCK");
      break;
    }

    case GO_TO_GOAL: {
      Serial.println("tring to go to the goal");
      // int distance = getSmoothedPingDistance();
      // while (distance < 10)
      // driveTowardYaw(0, 180);
      // break; 
      // Serial.println("GO_TO_GOAL");
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

void driveTowardYaw(float targetYaw, int baseSpeed = 150) {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float rawYaw = euler.x(); 
  float yaw = rawYaw - yawOffset;
  if (yaw < 0) yaw += 360;
  if (yaw >= 360) yaw -= 360;

  float yawError = targetYaw - yaw;
  if (yawError > 180) yawError -= 360;
  if (yawError < -180) yawError += 360;

  // 计算时间差
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0; // 转换为秒
  lastTime = currentTime;
  
  // 积分项
  integral += yawError * deltaTime;
  integral = constrain(integral, -50, 50); // 防止积分饱和
  
  // 微分项
  float derivative = (yawError - lastError) / deltaTime;
  lastError = yawError;
  
  // PID计算
  float correction = kP * yawError + kI * integral + kD * derivative;

  // 根据速度动态调整PID参数
  if (abs(yawError) < 10) {
    // 误差小时减小P增益，增加D增益以减小振荡
    correction = (kP * 0.5) * yawError + kI * integral + (kD * 1.5) * derivative;
  }

  int leftSpeed = constrain(baseSpeed - correction, 100, 255);
  int rightSpeed = constrain(baseSpeed + correction, 100, 255);

  motors.setM1Speed(rightSpeed);
  motors.setM2Speed(leftSpeed);

  Serial.print("🎯 Driving toward ");
  Serial.print(targetYaw);
  Serial.print("° | Current Yaw: ");
  Serial.print(yaw);
  Serial.print(" | Error: ");
  Serial.print(yawError);
  Serial.print(" | P: ");
  Serial.print(kP * yawError);
  Serial.print(" | I: ");
  Serial.print(kI * integral);
  Serial.print(" | D: ");
  Serial.println(kD * derivative);
}

bool turnWithPID(float targetAngle, int maxTurnSpeed = 150) {
  // 获取当前朝向
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float rawYaw = euler.x(); 
  float yaw = rawYaw - yawOffset;
  if (yaw < 0) yaw += 360;
  if (yaw >= 360) yaw -= 360;
  
  // 计算误差 (最短路径)
  float yawError = targetAngle - yaw;
  if (yawError > 180) yawError -= 360;
  if (yawError < -180) yawError += 360;
  
  // 如果已到达目标角度(±2度误差范围内)，则停止转向
  if (abs(yawError) < 2.0) {
    motors.setM1Speed(0);
    motors.setM2Speed(0);
    return true; // 转向完成
  }
  
  // 计算时间差
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0; // 转换为秒
  if (deltaTime > 0.5) deltaTime = 0.01; // 防止长时间暂停后的大跳变
  lastTime = currentTime;
  
  // PID控制器 - 专门为原地转向优化
  // 为转向设置不同的PID参数
  float turnKp = 2.5;  // 比直线行驶时更高的比例增益
  float turnKi = 0.1;  // 积分增益保持较低
  float turnKd = 1.0;  // 增加微分增益以减少振荡
  
  // 积分项
  integral += yawError * deltaTime;
  integral = constrain(integral, -30, 30); // 限制积分项，防止积分饱和
  
  // 微分项
  float derivative = (yawError - lastError) / deltaTime;
  lastError = yawError;
  
  // 计算PID输出
  float pidOutput = turnKp * yawError + turnKi * integral + turnKd * derivative;
  
  // 根据误差大小动态调整PID参数
  if (abs(yawError) < 10) {
    // 接近目标时，减小P增益，增大D增益以减小振荡
    pidOutput = (turnKp * 0.7) * yawError + turnKi * integral + (turnKd * 1.5) * derivative;
  }
  
  // 将PID输出转换为电机速度
  int turnSpeed = constrain(abs(pidOutput), 75, maxTurnSpeed);
  
  // 设置转向方向
  if (yawError > 0) { // 需要顺时针转向
    motors.setM1Speed(turnSpeed);    // 右电机正转
    motors.setM2Speed(-turnSpeed);   // 左电机反转
  } else { // 需要逆时针转向
    motors.setM1Speed(-turnSpeed);   // 右电机反转
    motors.setM2Speed(turnSpeed);    // 左电机正转
  }
  
  // 输出调试信息
  Serial.print("🔄 转向: 当前=");
  Serial.print(yaw);
  Serial.print("° 目标=");
  Serial.print(targetAngle);
  Serial.print("° 误差=");
  Serial.print(yawError);
  Serial.print("° 速度=");
  Serial.print(turnSpeed);
  Serial.print(" P=");
  Serial.print(turnKp * yawError);
  Serial.print(" I=");
  Serial.print(turnKi * integral);
  Serial.print(" D=");
  Serial.println(turnKd * derivative);
  
  return false; // 转向未完成
}

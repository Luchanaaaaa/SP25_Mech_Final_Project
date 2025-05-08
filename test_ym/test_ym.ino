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
int p=0;
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

// turing pid
float turnKp = 3.5;  // 转向的比例系数略高
float turnKi = 0.15;
float turnKd = 1.5;  // 转向的微分系数更高，减少振荡
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

// SubStates for drving toward a angle
enum DriveSubState {
  TURNING_TO_TARGET,   // 旋转阶段
  DRIVING_STRAIGHT     // 直行阶段
};
DriveSubState currentDriveSubState = TURNING_TO_TARGET;

// 电机平衡因子 - 用于补偿左右电机差异
float motorBalanceFactor = 1.0;  // 大于1表示左轮比右轮快，小于1表示右轮比左轮快

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
  
  // 初始化PID时间变量
  driveLastTime = millis();
  turnLastTime = millis();
  
  // 重置积分项
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
          Serial.println("角度对齐了");
          currentState = GO_TO_THE_PUCK;
        } else {
          Serial.println("❌角度没对齐");
          // 在切换到GET_BEHIND_PUCK状态前重置PID变量
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
      
      // 设置一个更大的预警距离，提前开始减速
      const int SLOW_DOWN_THRESHOLD = 30;  // 比WALL_DISTANCE_THRESHOLD大
      
      if (distance > SLOW_DOWN_THRESHOLD || pixySeesOrange()) {
        // 远离墙壁时正常速度
        driveTowardYaw(180, 150);
        Serial.println("🚗 Drving Toward Yaw we want!");
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
        
        // 在切换到TURN_TO_ANGLE状态前重置PID变量
        turnIntegral = 0.0;
        turnLastError = 0.0;
        turnLastTime = millis();
        
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
      int distance = readPingDistance();
      Serial.print("距离球: ");
      Serial.println(distance);
      
      // 检测是否已经碰到球
      if (distance <= 3 || distance > 300) {  // 非常近，说明已经接触到球
        Serial.println("✅ 已接触到球，准备对准球门");
        motors.setM1Speed(0);
        motors.setM2Speed(0);
        delay(200);  // 确保完全停止
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

void goToThePuck(int customSpeed = 150) {  
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
  static float lastTargetYaw = -999;  // 初始化为一个不可能的角度值
  
  // 如果目标角度发生变化，重置子状态
  if (targetYaw != lastTargetYaw) {
    currentDriveSubState = TURNING_TO_TARGET;
    lastTargetYaw = targetYaw;
    
    // 重置PID变量
    turnIntegral = 0.0;
    turnLastError = 0.0;
    turnLastTime = millis();
    
    driveIntegral = 0.0;
    driveLastError = 0.0;
    driveLastTime = millis();
    
    Serial.print("⚙️ 新的目标角度: ");
    Serial.println(targetYaw);
  }
  
  // 获取当前角度
  float yaw = getNormalizedYaw();
  
  // 执行当前子状态的动作
  if (currentDriveSubState == TURNING_TO_TARGET) {
    // 执行转向
    if (turnWithPID(targetYaw, 120)) {  // 使用较低的最大转向速度
      // 转向完成，切换到直行状态
      Serial.println("✅ 转向完成，开始直行");
      currentDriveSubState = DRIVING_STRAIGHT;
      
      // 重置直行PID变量
      driveIntegral = 0.0;
      driveLastError = 0.0;
      driveLastTime = millis();
      
      // 短暂停顿，确保稳定
      motors.setM1Speed(0);
      motors.setM2Speed(0);
      delay(200);
    }
  } else {
    // 执行直线行驶
    // 计算误差
    float yawError = targetYaw - yaw;
    if (yawError > 180) yawError -= 360;
    if (yawError < -180) yawError += 360;
    
    // 检查是否需要重新转向(如果偏离太多)
    if (abs(yawError) > 25.0) {
      Serial.println("⚠️ 偏离太多，切回转向状态");
      currentDriveSubState = TURNING_TO_TARGET;
      
      // 停止电机
      motors.setM1Speed(0);
      motors.setM2Speed(0);
      delay(100);
      
      // 重置转向PID变量
      turnIntegral = 0.0;
      turnLastError = 0.0;
      turnLastTime = millis();
      return;
    }
    
    // 计算时间差
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - driveLastTime) / 1000.0; // 转换为秒
    
    // 安全检查
    if (deltaTime <= 0 || deltaTime > 0.5) {
      deltaTime = 0.01; // 如果时间异常则使用安全默认值
    }
    
    driveLastTime = currentTime;
    
    // 积分项
    driveIntegral += yawError * deltaTime;
    driveIntegral = constrain(driveIntegral, -40, 40); // 防止积分饱和
    
    // 微分项
    float derivative = (yawError - driveLastError) / deltaTime;
    driveLastError = yawError;
    
    // PID计算 - 更积极的校正
    float correction = driveKp * yawError + driveKi * driveIntegral + driveKd * derivative;
  
    // 根据误差大小调整控制强度
    if (abs(yawError) < 5) {
      // 误差小时，使用较轻的校正
      correction = (driveKp * 0.6) * yawError + (driveKi * 0.8) * driveIntegral + (driveKd * 1.2) * derivative;
    } else if (abs(yawError) > 15) {
      // 误差大时，使用较强的校正
      correction = (driveKp * 1.3) * yawError + driveKi * driveIntegral + driveKd * derivative;
    }
  
    // 应用电机平衡因子
    float leftAdjustment = 0;
    float rightAdjustment = 0;
    
    if (motorBalanceFactor > 1.0) {
      // 左轮比右轮快
      leftAdjustment = baseSpeed * (motorBalanceFactor - 1.0);
    } else if (motorBalanceFactor < 1.0) {
      // 右轮比左轮快
      rightAdjustment = baseSpeed * (1.0 - motorBalanceFactor);
    }
  
    // 设置电机速度
    int leftSpeed = constrain(baseSpeed - correction + leftAdjustment, 70, 200);
    int rightSpeed = constrain(baseSpeed + correction + rightAdjustment, 70, 200);
  
    motors.setM1Speed(rightSpeed);
    motors.setM2Speed(leftSpeed);
  
    // 输出调试信息
    Serial.print("🚗 直行: 当前=");
    Serial.print(yaw);
    Serial.print("° 目标=");
    Serial.print(targetYaw);
    Serial.print("° 误差=");
    Serial.print(yawError);
    Serial.print("° P=");
    Serial.print(driveKp * yawError);
    Serial.print(" I=");
    Serial.print(driveKi * driveIntegral);
    Serial.print(" D=");
    Serial.print(driveKd * derivative);
    Serial.print(" 左速=");
    Serial.print(leftSpeed);
    Serial.print(" 右速=");
    Serial.println(rightSpeed);
  }
}
// void driveTowardYaw(float targetYaw, int baseSpeed = 150) {
//   imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
//   float yaw = euler.x(); 
//   yaw = yaw - yawOffset;
//   if (yaw < 0) yaw += 360;
//   if (yaw >= 360) yaw -= 360;

//   float yawError = targetYaw - yaw;
//   if (yawError > 180) yawError -= 360;
//   if (yawError < -180) yawError += 360;

//   // 计算时间差 - 使用驾驶专用时间变量
//   unsigned long currentTime = millis();
//   float deltaTime = (currentTime - driveLastTime) / 1000.0; // 转换为秒
  
//   // 添加时间安全检查
//   if (deltaTime <= 0 || deltaTime > 0.5) {
//     deltaTime = 0.01; // 如果时间异常则使用安全默认值
//   }
  
//   driveLastTime = currentTime;
  
//   // 积分项 - 使用驾驶专用积分变量
//   driveIntegral += yawError * deltaTime;
//   driveIntegral = constrain(driveIntegral, -50, 50); // 防止积分饱和
  
//   // 微分项 - 使用驾驶专用误差变量
//   float derivative = (yawError - driveLastError) / deltaTime;
//   driveLastError = yawError;
  
//   // PID计算 - 使用驾驶专用PID参数
//   float correction = driveKp * yawError + driveKi * driveIntegral + driveKd * derivative;

//   // 根据速度动态调整PID参数
//   if (abs(yawError) < 10) {
//     // 误差小时减小P增益，增加D增益以减小振荡
//     correction = (driveKp * 0.5) * yawError + driveKi * driveIntegral + (driveKd * 1.5) * derivative;
//   }

//   int leftSpeed = constrain(baseSpeed - correction, 100, 255);
//   int rightSpeed = constrain(baseSpeed + correction, 100, 255);

//   motors.setM1Speed(rightSpeed);
//   motors.setM2Speed(leftSpeed);

//   Serial.print("🎯 Driving toward ");
//   Serial.print(targetYaw);
//   Serial.print("° | Current Yaw: ");
//   Serial.print(yaw);
//   Serial.print(" | Error: ");
//   Serial.print(yawError);
//   Serial.print(" | P: ");
//   Serial.print(driveKp * yawError);
//   Serial.print(" | I: ");
//   Serial.print(driveKi * driveIntegral);
//   Serial.print(" | turnSpeedD: ");
//   Serial.println(driveKd * derivative);
// }

bool turnWithPID(float targetAngle, int maxTurnSpeed = 150) {
  const int minSpeed = 100;  // 调整这个值直到能在小误差时也能移动
  const int maxSpeed = 150;

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float yaw = euler.x(); 
  yaw = yaw - yawOffset;
  if (yaw < 0) yaw += 360;
  if (yaw >= 360) yaw -= 360;
  
  // 计算误差 (最短路径)
  float yawError = targetAngle - yaw;
  if (yawError > 180) yawError -= 360;
  if (yawError < -180) yawError += 360;
  
  // 恢复较小的误差阈值，提高精度
  if (abs(yawError) < 2.0) {
    motors.setM1Speed(0);
    motors.setM2Speed(0);
    return true; // 转向完成
  }
  
  // 计算时间差 - 使用转向专用时间变量
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - turnLastTime) / 1000.0; // 转换为秒
  
  // 添加时间安全检查
  if (deltaTime <= 0 || deltaTime > 0.5) {
    deltaTime = 0.01; // 如果时间异常则使用安全默认值
  }
  
  turnLastTime = currentTime;
  
  // 积分项 - 使用转向专用积分变量
  turnIntegral += yawError * deltaTime;
  turnIntegral = constrain(turnIntegral, -30, 30); // 限制积分项，防止积分饱和
  
  // 微分项 - 使用转向专用误差变量
  float derivative = (yawError - turnLastError) / deltaTime;
  turnLastError = yawError;
  
  // 计算PID输出 - 使用转向专用PID参数
  float pidOutput = turnKp * yawError + turnKi * turnIntegral + turnKd * derivative;
  
  // 根据误差大小动态调整PID参数
  if (abs(yawError) < 10) {
    // 接近目标时，减小P增益，增大D增益以减小振荡
    pidOutput = (turnKp * 0.7) * yawError + turnKi * turnIntegral + (turnKd * 1.5) * derivative;
  }
  
  int turnSpeed = constrain(abs(pidOutput), minSpeed, maxSpeed);

  if (abs(yawError) < 5.0 && abs(pidOutput) < 5.0) {
    // 到达目标，停止电机
    return true;
  }
  
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
  Serial.print(turnKi * turnIntegral);
  Serial.print(" D=");
  Serial.println(turnKd * derivative);
  
  return false; // 转向未完成
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

// 获取经过归一化处理的偏航角
float getNormalizedYaw() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float yaw = euler.x() - yawOffset;
  if (yaw < 0) yaw += 360;
  if (yaw >= 360) yaw -= 360;
  return yaw;
}
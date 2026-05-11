#include <AccelStepper.h>
#include <math.h>

// 硬件配置
#define PAN_STEP_PIN 16
#define PAN_DIR_PIN 17
#define TILT_STEP_PIN 18
#define TILT_DIR_PIN 5

// GPIO控制引脚
#define GPIO_25 25
#define GPIO_26 26
#define GPIO_27 27

// 机械参数
#define PAN_GEAR_RATIO 12.0
#define TILT_GEAR_RATIO 13.0
#define STEPS_PER_REV 200
#define MIN_TILT_ANGLE 0
#define MAX_TILT_ANGLE 60
#define TILT_ZERO_POS 30  // 俯仰轴零点位置

// 电机对象
AccelStepper panStepper(AccelStepper::DRIVER, PAN_STEP_PIN, PAN_DIR_PIN);
AccelStepper tiltStepper(AccelStepper::DRIVER, TILT_STEP_PIN, TILT_DIR_PIN);

// 角度状态
float currentPanAngle = 0;
float targetPanAngle = 0;
float targetTiltAngle = TILT_ZERO_POS;  // 初始化为零点位置

// GPIO状态
bool gpio25State = false;
bool gpio26State = false;
bool gpio27Pulse = false;
unsigned long gpio27PulseTime = 0;

float calculateShortestRotation(float current, float target) {
  float diff = fmod(target - current, 360);
  return (diff > 180) ? diff - 360 : (diff < -180) ? diff + 360 : diff;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);  // 等待串口连接
  
  // 电机参数设置
  panStepper.setMaxSpeed(1000);
  panStepper.setAcceleration(500);
  tiltStepper.setMaxSpeed(800);
  tiltStepper.setAcceleration(400);
  
  // 初始化位置
  panStepper.setCurrentPosition(0);
  tiltStepper.setCurrentPosition(
    (TILT_ZERO_POS * STEPS_PER_REV * TILT_GEAR_RATIO) / 360
  );
  
  // GPIO初始化
  pinMode(GPIO_25, OUTPUT);
  pinMode(GPIO_26, OUTPUT);
  pinMode(GPIO_27, OUTPUT);
  digitalWrite(GPIO_25, LOW);
  digitalWrite(GPIO_26, LOW);
  digitalWrite(GPIO_27, LOW);
  
  Serial.println("ESP32 Ready (Tilt Zero at 30°)");
}

void processSerial() {
  if (Serial.available() >= 10) {  // 现在数据包为10字节(4+4+1+1)
    byte buffer[10];
    Serial.readBytes(buffer, 10);
    
    if (buffer[9] == 0xFF) {
      // 解析角度数据 (Python发送的是放大100倍的值)
      targetPanAngle = (float)((buffer[0] << 24) | (buffer[1] << 16) | 
                              (buffer[2] << 8) | buffer[3]) / 100.0;
      targetTiltAngle = (float)((buffer[4] << 24) | (buffer[5] << 16) | 
                               (buffer[6] << 8) | buffer[7]) / 100.0;
      
      // 解析GPIO控制字节 (bit0:GPIO25, bit1:GPIO26, bit2:GPIO27脉冲)
      byte gpioByte = buffer[8];
      gpio25State = gpioByte & 0x01;
      gpio26State = gpioByte & 0x02;
      gpio27Pulse = gpioByte & 0x04;
      
      // 规范化角度
      targetPanAngle = fmod(targetPanAngle, 360);
      if (targetPanAngle < 0) targetPanAngle += 360;
      targetTiltAngle = constrain(targetTiltAngle, MIN_TILT_ANGLE, MAX_TILT_ANGLE);
      
      Serial.print("Target Pan: ");
      Serial.print(targetPanAngle);
      Serial.print("°, Tilt: ");
      Serial.print(targetTiltAngle);
      Serial.print("°, GPIO25:");
      Serial.print(gpio25State ? "HIGH" : "LOW");
      Serial.print(", GPIO26:");
      Serial.print(gpio26State ? "HIGH" : "LOW");
      Serial.print(", GPIO27:");
      Serial.println(gpio27Pulse ? "PULSE" : "LOW");
    }
  }
}

void updateMotors() {
  // 计算旋转轴步数(最短路径)
  float panRotation = calculateShortestRotation(currentPanAngle, targetPanAngle);
  long panSteps = panStepper.currentPosition() + 
                 (panRotation * STEPS_PER_REV * PAN_GEAR_RATIO) / 360;
  
  // 计算俯仰轴步数
  long tiltSteps = (targetTiltAngle * STEPS_PER_REV * TILT_GEAR_RATIO) / 360;
  
  // 设置目标位置
  panStepper.moveTo(panSteps);
  tiltStepper.moveTo(tiltSteps);
  
  // 运行电机
  panStepper.run();
  tiltStepper.run();
  
  // 更新当前角度
  currentPanAngle = fmod(
    (panStepper.currentPosition() * 360.0) / (STEPS_PER_REV * PAN_GEAR_RATIO), 
    360
  );
}

void updateGPIO() {
  // 更新GPIO25和GPIO26状态
  digitalWrite(GPIO_25, gpio25State ? HIGH : LOW);
  digitalWrite(GPIO_26, gpio26State ? HIGH : LOW);
  
  // 处理GPIO27脉冲
  if (gpio27Pulse) {
    digitalWrite(GPIO_27, HIGH);
    gpio27PulseTime = millis();
    gpio27Pulse = false;  // 复位脉冲标志
  }
  
  // 500ms后自动关闭GPIO27脉冲
  if (digitalRead(GPIO_27) == HIGH && (millis() - gpio27PulseTime) > 500) {
    digitalWrite(GPIO_27, LOW);
  }
}

void loop() {
  processSerial();
  updateMotors();
  updateGPIO();
}
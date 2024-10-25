#include <Servo.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);

// Servo pin assignments
#define SERVO_COXA_PIN 9
#define SERVO_FEMUR_PIN 10
#define SERVO_TIBIA_PIN 11

// Leg lengths
#define L1 70
#define L2 120
#define L3 193

Servo servoCoxa, servoFemur, servoTibia;

void setup() {
  Serial.begin(57600);

  servoCoxa.attach(SERVO_COXA_PIN);
  servoFemur.attach(SERVO_FEMUR_PIN);
  servoTibia.attach(SERVO_TIBIA_PIN);

  pwm.begin();
  pwm2.begin();
  pwm.setPWMFreq(60);
  pwm2.setPWMFreq(60);
}

// 보행 궤적 함수: 다리의 시작과 끝 좌표, 호버링 높이, 이동 시간을 설정하여 다리의 궤적을 생성
void moveLegTrajectory(float startX, float startY, float startZ,
                       float endX, float endY, float endZ,
                       float hoverHeight, float duration) {
  int steps = 50;  // 궤적을 나눌 스텝 수
  float timeStep = duration / steps;

  // 첫 번째 단계: 직선 운동 (발을 내딛기)
  for (int i = 0; i < steps / 2; i++) {
    float t = (float)i / (steps / 2);
    float x = startX + (endX - startX) * t;
    float y = startY + (endY - startY) * t;
    float z = startZ;  // 직선 운동이므로 높이는 일정

    // 다리 위치 이동 함수 호출
    moveLegTo(x, y, z);
    delay(timeStep * 1000);
  }

  // 두 번째 단계: 포물선 운동 (발을 다시 앞으로 가져오기)
  for (int i = steps / 2; i <= steps; i++) {
    float t = (float)(i - steps / 2) / (steps / 2);
    float x = endX + (startX - endX) * t;
    float y = endY + (startY - endY) * t;
    float z = startZ + hoverHeight * (1 - pow(2 * t - 1, 2));  // 포물선 궤적

    // 다리 위치 이동 함수 호출
    moveLegTo(x, y, z);
    delay(timeStep * 1000);
  }
}


// 다리를 특정 좌표로 이동
void moveLegTo(float targetX, float targetY, float targetZ) {
  float offsetX = targetX;
  float offsetY = targetY;
  float offsetZ = targetZ;

  float coxaAngle = atan2(offsetY, offsetX) * 180 / PI;
  float d1 = sqrt(offsetX * offsetX + offsetY * offsetY) - L1;
  float d2 = sqrt(d1 * d1 + offsetZ * offsetZ);

  float femurAngle = atan2(offsetZ, d1) * 180 / PI + acos((L2 * L2 + d2 * d2 - L3 * L3) / (2 * L2 * d2)) * 180 / PI;
  float tibiaAngle = acos((L2 * L2 + L3 * L3 - d2 * d2) / (2 * L2 * L3)) * 180 / PI;

  servoCoxa.write(90 + coxaAngle);
  servoFemur.write(femurAngle);
  servoTibia.write(180 - tibiaAngle);

  Serial.print("servoCoxa = ");
  Serial.print(90 + coxaAngle);
  Serial.print(", servoFemur = ");
  Serial.print(femurAngle);
  Serial.print(", servoTibia = ");
  Serial.print(180 - tibiaAngle);
  Serial.println();
}

void loop() {
  // Tripod Gait 궤적을 설정하여 다리 이동
  moveLegTrajectory(-50, 50, 0, -50, 70, 0, 50, 1.0);  // 예제: 시작, 끝 좌표, 호버링 높이, 이동 시간
  delay(1000);
}

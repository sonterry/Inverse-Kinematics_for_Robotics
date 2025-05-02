#include <Servo.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);


// 6개의 다리마다 3개의 서보 (coxa, femur, tibia)
Servo servo_coxa[6];
Servo servo_femur[6];
Servo servo_tibia[6];

// 각 서보 핀 번호
const int coxaPins[6] = {2, 5, 8, 11, 14, 17};
const int femurPins[6] = {3, 6, 9, 12, 15, 18};
const int tibiaPins[6] = {4, 7, 10, 13, 16, 19};

// 각 서보 오프셋 (기본 90°, 필요하면 수정)
const int coxaOffsets[6] = {90, 90, 90, 90, 90, 90};
const int femurOffsets[6] = {90, 90, 90, 90, 90, 90};
const int tibiaOffsets[6] = {90, 90, 90, 90, 90, 90};

// 각 서보 회전 방향 (1 또는 -1)
const int coxaDirs[6] = {1, 1, 1, 1, 1, 1};
const int femurDirs[6] = {1, 1, 1, 1, 1, 1};
const int tibiaDirs[6] = {1, 1, 1, 1, 1, 1};

void motor(int Number,int Angle)
{
  //if (Angle > 270 || Angle < 0) {Angle = 135;}  // 방어코드 추가
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  if (Number <= 16)
  {
    pwm.setPWM(Number, 0, a);
  }
  else if (Number >= 17)
  {
    pwm2.setPWM(Number-16, 0, a);
  }
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 6; i++) {
    servo_coxa[i].attach(coxaPins[i]);
    servo_femur[i].attach(femurPins[i]);
    servo_tibia[i].attach(tibiaPins[i]);
  }

   // ----- 초기 보정: 모든 서보를 90도로 이동 -----
  for (int i = 0; i < 6; i++) {
    servo_coxa[i].write(90);
    servo_femur[i].write(90);
    servo_tibia[i].write(90);
  }

  delay(5000); // 5초 동안 90도 자세 유지 (조립 상태 눈으로 확인)
}

void loop() {
  if (Serial.available() >= 18) { // 18바이트 수신
    for (int i = 0; i < 6; i++) {
      int raw_theta0 = Serial.read(); // coxa
      int raw_theta1 = Serial.read(); // femur
      int raw_theta2 = Serial.read(); // tibia

      int final_theta0 = constrain(coxaOffsets[i] + coxaDirs[i] * (raw_theta0 - 90), 0, 180);
      int final_theta1 = constrain(femurOffsets[i] + femurDirs[i] * (raw_theta1 - 90), 0, 180);
      int final_theta2 = constrain(tibiaOffsets[i] + tibiaDirs[i] * (raw_theta2 - 90), 0, 180);

      //servo_coxa[i].write(final_theta0);
      //servo_femur[i].write(final_theta1);
      //servo_tibia[i].write(final_theta2);
    }
  }
}

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ----- PCA9685 드라이버 두 개 선언 -----
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);

// ----- 6개의 다리마다 3개의 서보 (coxa, femur, tibia) -----

// PWM 채널 번호 (0~31)
const int coxaPins[6] = {0, 3, 6, 9, 12, 17};   
const int femurPins[6] = {1, 4, 7, 10, 13, 18};
const int tibiaPins[6] = {2, 5, 8, 11, 14, 19};

// ----- 각 서보 오프셋 (기본 135°) -----
const int coxaOffsets[6] = {135, 135, 135, 135, 135, 135};
const int femurOffsets[6] = {135, 135, 135, 135, 135, 135};
const int tibiaOffsets[6] = {135, 135, 135, 135, 135, 135};

// ----- 각 서보 회전 방향 (1 또는 -1) -----
const int coxaDirs[6] = {1, 1, 1, 1, 1, 1};
const int femurDirs[6] = {1, 1, 1, 1, 1, 1};
const int tibiaDirs[6] = {1, 1, 1, 1, 1, 1};

// ----- motor 함수 (유지) -----
void motor(int Number, int Angle)
{
  if (Angle > 270 || Angle < 0) 
  {
    return; // 잘못된 값이면 아무 것도 하지 않고 종료
  }

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

void setup() 
{
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm2.begin();

  pwm.setPWMFreq(50);  // 50Hz 주파수
  pwm2.setPWMFreq(50);

  delay(10);

  // ----- 초기 보정: 모든 서보를 135도로 이동 -----
  for (int i = 0; i < 6; i++) 
  {
    motor(coxaPins[i], 90);
    motor(femurPins[i], 90);
    motor(tibiaPins[i], 90);
  }

  delay(600000); // 1분 동안 135도 자세 유지 (조립 상태 눈으로 확인)
}

void loop() 
{
  if (Serial.available() >= 18) 
  { // 18바이트 수신
    for (int i = 0; i < 6; i++) 
    {
      int raw_theta0 = Serial.read(); // coxa
      int raw_theta1 = Serial.read(); // femur
      int raw_theta2 = Serial.read(); // tibia

      int final_theta0 = constrain(coxaOffsets[i] + coxaDirs[i] * (raw_theta0 - 135), 0, 270);
      int final_theta1 = constrain(femurOffsets[i] + femurDirs[i] * (raw_theta1 - 135), 0, 270);
      int final_theta2 = constrain(tibiaOffsets[i] + tibiaDirs[i] * (raw_theta2 - 135), 0, 270);

      motor(coxaPins[i], final_theta0);
      motor(femurPins[i], final_theta1);
      motor(tibiaPins[i], final_theta2);
    }
  }
}

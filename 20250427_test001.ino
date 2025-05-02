#include <Servo.h>

Servo myServo;
int angle = 0;

void setup() {
  Serial.begin(9600);
  myServo.attach(9); // 서보를 9번 핀에 연결
}

void loop() {
  if (Serial.available()) {
    angle = Serial.parseInt(); // 파이썬에서 숫자를 받아서
    myServo.write(angle);       // 서보를 그 각도로 움직임
  }
}

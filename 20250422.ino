#include <Servo.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);

// Servo pin assignments
#define LEG_4_SERVO_COXA_PIN 9
#define LEG_4_SERVO_FEMUR_PIN 10
#define LEG_4_SERVO_TIBIA_PIN 11

// Leg lengths (mm)
#define L1 70   // Coxa
#define L2 120  // Femur
#define L3 193  // Tibia

// Leg 4 base position (for future use)
#define LEG_4_PELVIS_X 0
#define LEG_4_PELVIS_Y 0
#define LEG_4_PELVIS_Z 0

// Servo PWM control function (0~270deg mapped to pulse)
void motor(int Number, int Angle) {
  if (Angle > 270 || Angle < 0) {
    Angle = 135; // Neutral safety
  }
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  if (Number <= 16) {
    pwm.setPWM(Number, 0, a);
  } else {
    pwm2.setPWM(Number - 16, 0, a);
  }
}



void moveLegTo(int LegNum, float targetX, float targetY, float targetZ) {
  if (LegNum != 4) return;

  Serial.println("----- moveLegTo Debug Start -----");
  Serial.print("Input Target: X="); Serial.print(targetX);
  Serial.print(", Y="); Serial.print(targetY);
  Serial.print(", Z="); Serial.println(targetZ);

  // Z축 반전 (시뮬레이터 기준)
  float z2 = -targetZ;

  // θ₀ (Coxa 회전)
  float theta0_rad = atan2(targetX, z2);
  float theta0_deg = degrees(theta0_rad);
  Serial.print("theta0_rad: "); Serial.println(theta0_rad, 6);
  Serial.print("theta0_deg: "); Serial.println(theta0_deg, 6);

  if (theta0_deg < -90.0 || theta0_deg > 90.0) {
    Serial.println("θ0 out of range");
    return;
  }

  // 평면 거리 및 2D 변환 (coxa 기준)
  float planarDist = sqrt(targetX * targetX + z2 * z2);
  float x2d = planarDist - L1;

  // ✅ Y축 반전 (시뮬레이터와 일치)
  float y2d = -targetY;

  Serial.print("planarDist: "); Serial.println(planarDist, 6);
  Serial.print("x2d = planarDist - L1: "); Serial.println(x2d, 6);
  Serial.print("y2d: "); Serial.println(y2d, 6);

  // θ₂ 계산 (Elbow-Down)
  float d = sqrt(x2d * x2d + y2d * y2d);
  float cos_theta2 = (x2d * x2d + y2d * y2d - L2 * L2 - L3 * L3) / (2.0 * L2 * L3);
  Serial.print("raw cos_theta2: "); Serial.println(cos_theta2, 6);
  cos_theta2 = constrain(cos_theta2, -1.0, 1.0);
  float theta2_rad = -acos(cos_theta2);
  float theta2_deg = degrees(theta2_rad);
  Serial.print("theta2_rad: "); Serial.println(theta2_rad, 6);
  Serial.print("theta2_deg: "); Serial.println(theta2_deg, 6);

  // θ₁ 계산
  float k1 = L2 + L3 * cos(theta2_rad);
  float k2 = L3 * sin(theta2_rad);
  float theta1_rad = atan2(y2d, x2d) - atan2(k2, k1);
  float theta1_deg = degrees(theta1_rad);
  Serial.print("k1: "); Serial.println(k1, 6);
  Serial.print("k2: "); Serial.println(k2, 6);
  Serial.print("theta1_rad: "); Serial.println(theta1_rad, 6);
  Serial.print("theta1_deg: "); Serial.println(theta1_deg, 6);

  // 최종 각도 출력
  Serial.print("θ0 = "); Serial.print(theta0_deg, 2);
  Serial.print("°, θ1 = "); Serial.print(theta1_deg, 2);
  Serial.print("°, θ2 = "); Serial.print(theta2_deg, 2);
  Serial.println("°");

  // 서보 각도로 변환 (중립 135도 기준)
  float angle0 = constrain(theta0_deg, 0.0, 270.0);
  float angle1 = constrain(135.0 - theta1_deg, 0.0, 270.0);
  float angle2 = constrain(135.0 - theta2_deg, 0.0, 270.0);

  Serial.print("Servo angles → ");
  Serial.print("Coxa: "); Serial.print(angle0);
  Serial.print(" | Femur: "); Serial.print(angle1);
  Serial.print(" | Tibia: "); Serial.println(angle2);

  motor(LEG_4_SERVO_COXA_PIN, angle0);
  motor(LEG_4_SERVO_FEMUR_PIN, angle1);
  motor(LEG_4_SERVO_TIBIA_PIN, angle2);

  Serial.println("----- moveLegTo Debug End -----");
}











void setup() {
  Serial.begin(57600);

  pwm.begin();
  pwm2.begin();
  pwm.setPWMFreq(60);
  pwm2.setPWMFreq(60);

  Serial.println("Ready to receive XYZ coords.");
}

void loop() {
  if (Serial.available() >= 3) {
    int x = Serial.parseInt();
    int y = Serial.parseInt();
    int z = Serial.parseInt();

    Serial.print("Input: ");
    Serial.print(x); Serial.print(", ");
    Serial.print(y); Serial.print(", ");
    Serial.println(z);

    moveLegTo(4, x, y, z);
  }
}

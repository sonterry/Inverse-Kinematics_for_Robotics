/*
Using the inverse kinematics used in the 6-legged hexabot robot with Arduino using the C language, hold the pelvis as the origin with three motors and input the coordinates for the toes, and then move the toes with those coordinates.
The leg to be tested is leg 3, and the motor between the body and l1 is connected to pin 9. The motor between l1 and l2 is connected to pin 10. The motor between l2 and l3 is connected to pin 11.
Code to move one leg in hexabot
The range of movement of the motor is from 0 to 180 degrees, and when the legs are stretched, they are all 90 degrees.
Each servo angle when the robot's legs are stretched downward is servoCoxa = 90 degrees, servoFemur = 0 degrees, servoTibia = 90 degrees
Display the angle of each servo in the serial window.
1. The leg length is l1=70cm, l2=120cm, l3=193cm
2. The coordinates of the torso are 0,0,0, and the coordinates of each leg are set separately.
3.  x-axis for side, y-axis for front and back, and z-axis for up and down
4. The pelvic coordinates of the first leg are -40,60,0
   The pelvic coordinates of the second leg are 40,60,0
   The pelvic coordinates of the third leg are -60,0,0
   The pelvic coordinates of the fourth leg are 60,0,0
   The pelvic coordinates of the fifth leg are -40, -60,0
   The pelvic coordinates of the 6th leg are 40,-60,0

example :
If you move the toe to 383,0,0, servoCoxa = 90 degrees, servoFemur = 90 degrees, and servoTibia = 90 degrees.
If you move the toe to 0,383,0, servoCoxa = 0 degrees, servoFemur = 90 degrees, and servoTibia = 90 degrees.
If you move the toe to 70,0,-313, servoCoxa = 90 degrees, servoFemur = 0 degrees, and servoTibia = 90 degrees.
If you move the toe to 70,0,313, servoCoxa = 90 degrees, servoFemur = 180 degrees, and servoTibia = 90 degrees.
If you move the toe to 190,0,193, servoCoxa = 90 degrees, servoFemur = 90 degrees, and servoTibia = 180 degrees.
If you move the toe to 190,0,-193, servoCoxa = 90 degrees, servoFemur = 90 degrees, and servoTibia = 0 degrees.
*/

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);


//#define PI 3.14

// Servo pin assignments
#define SERVO_COXA_PIN 9
#define SERVO_FEMUR_PIN 10
#define SERVO_TIBIA_PIN 11

// Leg lengths
#define L1 70
#define L2 120
#define L3 193

// Leg 3 pelvic coordinates
#define LEG_3_PELVIS_X 0
#define LEG_3_PELVIS_Y 0
#define LEG_3_PELVIS_Z 0

Servo servoCoxa, servoFemur, servoTibia;



void motor(int Number,int Angle)
{
  if (Angle > 270 || Angle < 0) {Angle = 135;}  // 방어코드 추가
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

void test()
{
  motor(9,135);
  Serial.println("135");
  delay(2000);
  motor(9,0);
  Serial.println("0");
  delay(400);



  
  /*for (int i = 0; i < 18; i++)
  {
    motor(0,135);
    Serial.println("135");
  }
  delay(2000);
  for (int i = 0; i < 18; i++)
  {
    motor(0,0);
    Serial.println("0");
  }
  delay(400);*/
}


void setup() 
{
  Serial.begin(57600);

  servoCoxa.attach(SERVO_COXA_PIN);
  servoFemur.attach(SERVO_FEMUR_PIN);
  servoTibia.attach(SERVO_TIBIA_PIN);
  
  pwm.begin();
  pwm2.begin();
  pwm.setPWMFreq(60);
  pwm2.setPWMFreq(60);
  Serial.println("SETUP");
}

void loop() 
{
  //test();
  if (Serial.available() >= 3) 
  {
    int x = Serial.parseInt();
    int y = Serial.parseInt();
    int z = Serial.parseInt();
    Serial.print(x);
    Serial.print("  ");
    Serial.print(y);
    Serial.print("  ");
    Serial.println(z);
    moveLegTo(x, y, z);
  }
//motor(9, 90); motor(10, 90); motor(11, 90);
  //motor(0, 90);
  
}

void moveLeg1(float targetX, float targetY, float targetZ) 
{
  float x, y, z;

  // Assuming x, y, and z are somehow provided by your system

  float d = sqrt(x * x + y * y - L1 * L1);
  float alpha = atan2(d, z);

  float servoCoxaAngle = atan2(y, x) * 180 / PI; 
  float servoFemurAngle = (alpha + acos((L2 * L2 + d * d - L3 * L3) / (2 * L2 * d))) * 180 / PI;
  float servoTibiaAngle = acos((L2 * L2 + L3 * L3 - d * d) / (2 * L2 * L3)) * 180 / PI;

  servoCoxa.write(servoCoxaAngle);
  servoFemur.write(servoFemurAngle);
  servoTibia.write(servoTibiaAngle);

  Serial.print("ServoCoxa: ");
  Serial.println(servoCoxaAngle);
  Serial.print("ServoFemur: ");
  Serial.println(servoFemurAngle);
  Serial.print("ServoTibia: ");
  Serial.println(servoTibiaAngle);
  
}

void moveLegTo(float targetX, float targetY, float targetZ) 
{
  float offsetX = targetX - LEG_3_PELVIS_X;
  float offsetY = targetY - LEG_3_PELVIS_Y;
  float offsetZ = targetZ - LEG_3_PELVIS_Z;

  float coxaAngle = atan2(offsetY, offsetX) * 180 / PI;
  float d1 = sqrt(offsetX * offsetX + offsetY * offsetY) - L1;
  float d2 = sqrt(d1 * d1 + offsetZ * offsetZ);

  if (d2 == 0) 
  {
    float femurAngle = 0;
    float tibiaAngle = 0;
    Serial.print("ERROR");
  } 
  else 
  {
  
    float a1 = atan2(offsetZ, d1);
    float a2 = acos(max(min((L2 * L2 + d2 * d2 - L3 * L3) / (2 * L2 * d2), 1.0), -1.0));
    float femurAngle = (a1 + a2) * 180 / PI;

    float a3 = acos(max(min((L2 * L2 + L3 * L3 - d2 * d2) / (2 * L2 * L3), 1.0), -1.0));
    float tibiaAngle = (PI - a3) * 180 / PI;

    motor(9,90+coxaAngle+45);
    motor(10,(femurAngle+90)+45);
    motor(11,(tibiaAngle-90)+45);
    
    Serial.print("servoCoxa = ");
    Serial.print(90+coxaAngle+45);
    Serial.print(" degrees, servoFemur = ");
    Serial.print((femurAngle+90)+45);
    Serial.print(" degrees, servoTibia = ");
    Serial.print((tibiaAngle-90)+45);
    Serial.println(" degrees");
  }
}

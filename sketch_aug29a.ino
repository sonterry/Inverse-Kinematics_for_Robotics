#include <Servo.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#define PI 3.14

Adafruit_PWMServoDriver pwm=Adafruit_PWMServoDriver();



Servo myservo;  // create servo object to control a servo
Servo myservo2;  // create servo object to control a servo

float L1 = 76;
float L2 = 127;

int X = (int)(L1+L2);
int Y = 0;

float thetaE = 0;
float thetaQ = 0;
float thetaS = 0;


void motor0(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(0,0,a);
}

void motor1(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(1,0,a);
}

void motor2(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(2,0,a);
}

void motor3(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(3,0,a);
}

void motor4(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(4,0,a);
}

void motor5(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(5,0,a);
}

void motor6(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(7,0,a);
}

void motor7(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(7,0,a);
}

void motor8(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(8,0,a);
}

void motor9(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(9,0,a);
}

void motor10(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(10,0,a);
}

void motor11(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(11,0,a);
}

void motor12(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(12,0,a);
}

void motor13(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(13,0,a);
}

void motor14(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(14,0,a);
}

void motor15(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(15,0,a);
}

void motor16(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(16,0,a);
}

void motor17(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(17,0,a);
} //여기까지 다리6개, 모터18개 


void lag1(int X,int Y)
{
  
      float X_max = (L1+L2);
      float X_min = (L1-L2) * cos(PI/4);
      float Y_max = (L1+L2);
      float Y_min = (L1-L2) * sin(PI/4);

      X = constrain(X,X_min,X_max);
      Y = constrain(Y,Y_min,Y_max);

      if (Serial.read() == '\n') {
        thetaE = acos((pow(X,2)+pow(Y,2)-pow(L1,2)-pow(L2,2))/(2*L1*L2));
        thetaQ = acos((pow(X,2)+pow(Y,2)+pow(L1,2)-pow(L2,2))/(2*L1*(sqrt((pow(X,2)+pow(Y,2))))));
        thetaS = atan2(Y,X) - thetaQ;

        Serial.print(thetaE * 180 / PI);
        Serial.print(",");
        Serial.println(thetaS * 180 / PI);

        //myservo.write(thetaS * 180 / PI+45);              // tell servo to go to position in variable 'pos'
        //myservo2.write(thetaE * 180 / PI+45);              // tell servo to go to position in variable 'pos'
        motor0((thetaS * 180 / PI+45));
        float m1 = thetaE * 180 / PI+45;
        motor1(180-m1);
}
}


void 1_stap_test(){
  
      for (float i = 0; i < 120; i=i+2){

      Y = 160;
      X = i;
      lag1(X,Y);

      delayMicroseconds(1);
    
    }delay(150);
      for (float i = 160; i > 1200; i=i-5){
      Y = i;
      X = 120;
      lag1(X,Y);

      delayMicroseconds(1);
    
    }delay(10);    
      for (float i = 120; i > 0; i=i-41){

      Y = 120;
      X = i;
      lag1(X,Y);

      delayMicroseconds(1);
    
    }delay(10);
    for (float i = 120; i < 160; i=i+5){

      Y = i;
      X = 0;
      lag1(X,Y);

      delayMicroseconds(1);
    
    }delay(150);

}






void setup() {

    pwm.begin();
    pwm.setPWMFreq(60);
    Serial.print('start');

    
    // put your setup code here, to run once:
    Serial.begin(9600);
    myservo.attach(9);  // attaches the servo on pin 9 to the servo object
    myservo2.attach(10);  // attaches the servo on pin 9 to the servo obje


        thetaE = acos((pow(X,2)+pow(Y,2)-pow(L1,2)-pow(L2,2))/(2*L1*L2));
        thetaQ = acos((pow(X,2)+pow(Y,2)+pow(L1,2)-pow(L2,2))/(2*L1*(sqrt((pow(X,2)+pow(Y,2))))));
        thetaS = atan2(Y,X) - thetaQ;

        Serial.print(thetaE * 180 / PI);
        Serial.print(",");
        Serial.println(thetaS * 180 / PI);

        myservo.write(thetaS * 180 / PI);              // tell servo to go to position in variable 'pos'
        myservo2.write(thetaE * 180 / PI);              // tell servo to go to position in variable 'pos'
        Serial.println("SETUP");
}





void loop() {
    while (Serial.available() > 0) {
          int X1 = Serial.parseInt();
          int Y1 = Serial.parseInt();
          lag1(X1,Y1);

    }
}

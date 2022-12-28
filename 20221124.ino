/////////////////////////////////////////////////////////////////
/////////////          ////         //////    //////  ///////////
////////////  /////////////  /////  //////  /  /////  ///////////
///////////  //////////////  /////  //////  //  ////  ///////////
//////////          ///////  /////  //////  ///  ///  ///////////
/////////////////  ////////  /////  //////  ////  //  ///////////
////////////////  /////////  /////  //////  /////  /  ///////////
///////          //////////         //////  //////    ///////////
/////////////////////////////////////////////////////////////////

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#define PI 3.14

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);

Servo myservo;  // create servo object to control a servo
Servo myservo2;  // create servo object to control a servo

float L1 = 295;
float L2 = 195;

int h=460;

int X = (int)(L1 + L2);
int Y = 0;

float thetaE = 0;
float thetaQ = 0;
float thetaS = 0;

int foot0 = 0;
int foot1 = 0;
int foot2 = 0;
int foot3 = 0;
int foot4 = 0;
int foot5 = 0;

float cos45 = 0.52532198881;
float sin45 = 0.85090352453;


void motor0(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(0, 0, a);
}

void  motor1(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(1, 0, a);
}

void motor2(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(2, 0, a);
}

void motor3(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(3, 0, a);
}

void motor4(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(4, 0, a);
}

void motor5(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(5, 0, a);
}

void motor6(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(6, 0, a);
}

void motor7(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(7, 0, a);
}

void motor8(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(8, 0, a);
}

void motor9(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(9, 0, a);
}

void motor10(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(10, 0, a);
}

void motor11(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(11, 0, a);
}

void motor12(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(12, 0, a);
}

void motor13(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(13, 0, a);
}

void motor14(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(14, 0, a);
}

void motor15(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(15, 0, a);
}

void motor16(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm2.setPWM(0, 0, a);
}

void motor17(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm2.setPWM(1, 0, a);
} //여기까지 다리6개, 모터18개


void lag0(float X1, float Y1)
{
  X = X1*cos45+Y1*sin45;
  Y = Y1*cos45-X1*sin45;

  float X_max = (L1 + L2);
  float X_min = (L1 - L2) * cos(PI / 4);
  float Y_max = (L1 + L2);
  float Y_min = (L1 - L2) * sin(PI / 4);

  X = constrain(X, X_min, X_max);
  Y = constrain(Y, Y_min, Y_max);

  thetaE = acos((pow(X, 2) + pow(Y, 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2));
  thetaQ = acos((pow(X, 2) + pow(Y, 2) + pow(L1, 2) - pow(L2, 2)) / (2 * L1 * (sqrt((pow(X, 2) + pow(Y, 2))))));
  thetaS = atan2(Y, X) - thetaQ;

  float m1 = thetaS * 180 / PI + 45;
  float m2 = thetaE * 180 / PI + 45;

  motor0 (m1+45);
  motor1(233);//235
  motor2(180-m2);
  
  Serial.print("  L0 ");
  Serial.print(m1+45);
  Serial.print(",");
  Serial.print(180-m2);
}

void lag1(float X1, float Y1)
{
  X = X1*cos45+Y1*sin45;
  Y = Y1*cos45-X1*sin45;

  float X_max = (L1 + L2);
  float X_min = (L1 - L2) * cos(PI / 4);
  float Y_max = (L1 + L2);
  float Y_min = (L1 - L2) * sin(PI / 4);

  X = constrain(X, X_min, X_max);
  Y = constrain(Y, Y_min, Y_max);

  thetaE = acos((pow(X, 2) + pow(Y, 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2));
  thetaQ = acos((pow(X, 2) + pow(Y, 2) + pow(L1, 2) - pow(L2, 2)) / (2 * L1 * (sqrt((pow(X, 2) + pow(Y, 2))))));
  thetaS = atan2(Y, X) - thetaQ;

  float m1 = thetaS * 180 / PI + 45;
  float m2 = thetaE * 180 / PI + 45;

  motor3 (270 - m1);
  motor4(37);//35
  motor5(180-m2);

  Serial.print("  L1 ");
  Serial.print(225 - m1);
  Serial.print(",");
  Serial.print(180-m2);
}

void lag2(float Y1, float X1)
{
  X = Y1*cos45+X1*sin45;
  Y = X1*cos45-Y1*sin45;

  float X_max = (L1 + L2);
  float X_min = (L1 - L2) * cos(PI / 4);
  float Y_max = (L1 + L2);
  float Y_min = (L1 - L2) * sin(PI / 4);

  X = constrain(X, X_min, X_max);
  Y = constrain(Y, Y_min, Y_max);

  thetaE = acos((pow(X, 2) + pow(Y, 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2));
  thetaQ = acos((pow(X, 2) + pow(Y, 2) + pow(L1, 2) - pow(L2, 2)) / (2 * L1 * (sqrt((pow(X, 2) + pow(Y, 2))))));
  thetaS = atan2(Y, X) - thetaQ;

  float m1 = thetaS * 180 / PI + 45;
  float m2 = thetaE * 180 / PI + 45;

  motor6 (m1+45);
  motor7(233);//235
  motor8(180-m2);
  
  Serial.print("  L2 ");
  Serial.print(m1+45);
  Serial.print(",");
  Serial.print(180-m2);
}

void lag3(float X1, float Y1)
{
  X = X1*cos45+Y1*sin45;
  Y = Y1*cos45-X1*sin45;

  float X_max = (L1 + L2);
  float X_min = (L1 - L2) * cos(PI / 4);
  float Y_max = (L1 + L2);
  float Y_min = (L1 - L2) * sin(PI / 4);

  X = constrain(X, X_min, X_max);
  Y = constrain(Y, Y_min, Y_max);

  thetaE = acos((pow(X, 2) + pow(Y, 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2));
  thetaQ = acos((pow(X, 2) + pow(Y, 2) + pow(L1, 2) - pow(L2, 2)) / (2 * L1 * (sqrt((pow(X, 2) + pow(Y, 2))))));
  thetaS = atan2(Y, X) - thetaQ;

  float m1 = thetaS * 180 / PI + 45;
  float m2 = thetaE * 180 / PI + 45;

  motor9 (225-m1-14);
  motor10(37);//35
  motor11(180-m2);
  
  Serial.print("  L3 ");
  Serial.print(180-m1-14);
  Serial.print(",");
  Serial.print(180-m2);
}

void lag4(float X1, float Y1)
{
  X = X1*cos45+Y1*sin45;
  Y = Y1*cos45-X1*sin45;

  float X_max = (L1 + L2);
  float X_min = (L1 - L2) * cos(PI / 4);
  float Y_max = (L1 + L2);
  float Y_min = (L1 - L2) * sin(PI / 4);

  X = constrain(X, X_min, X_max);
  Y = constrain(Y, Y_min, Y_max);

  thetaE = acos((pow(X, 2) + pow(Y, 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2));
  thetaQ = acos((pow(X, 2) + pow(Y, 2) + pow(L1, 2) - pow(L2, 2)) / (2 * L1 * (sqrt((pow(X, 2) + pow(Y, 2))))));
  thetaS = atan2(Y, X) - thetaQ;

  float m1 = thetaS * 180 / PI + 45;
  float m2 = thetaE * 180 / PI + 45;

  motor12 (m1+45);
  motor13(233);//235
  motor14(180-m2);

  Serial.print("  L4 ");
  Serial.print(m1+45);
  Serial.print(",");
  Serial.print(180-m2);
}

void lag5(float X1, float Y1)
{
  X = X1*cos45+Y1*sin45;
  Y = Y1*cos45-X1*sin45;

  float X_max = (L1 + L2);
  float X_min = (L1 - L2) * cos(PI / 4);
  float Y_max = (L1 + L2);
  float Y_min = (L1 - L2) * sin(PI / 4);

  X = constrain(X, X_min, X_max);
  Y = constrain(Y, Y_min, Y_max);

  thetaE = acos((pow(X, 2) + pow(Y, 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2));
  thetaQ = acos((pow(X, 2) + pow(Y, 2) + pow(L1, 2) - pow(L2, 2)) / (2 * L1 * (sqrt((pow(X, 2) + pow(Y, 2))))));
  thetaS = atan2(Y, X) - thetaQ;

  float m1 = thetaS * 180 / PI + 45;
  float m2 = thetaE * 180 / PI + 45;

  motor15 (225-m1);
  motor16(37);//35
  motor17(180-m2);

  Serial.print("  L5 ");
  Serial.print(225-m1);
  Serial.print(",");
  Serial.println(180-m2);
}


void test()
{
  motor0(135);
  motor1(135);
  motor2(135);
  motor3(135);
  motor4(135);
  motor5(135);
  motor6(135);
  motor7(135);
  motor8(135);
  motor9(135);
  motor10(135);
  motor11(135);
  motor12(135);
  motor13(135);
  motor14(135);
  motor15(135);
  motor16(135);
  motor17(135);
  delay(999999999999);
  motor17(135);
}


/*
  int Step_Measurement(int X1,int Y1)
  {
    lag1(X1,Y1);
    delay(200);
    lag1(X1,Y1+50);
    for(int i=X1;i>j;i++)
    {
        lag1(i,Y1+50);
        if(foot1>=1)
        {
            public:int wall1 = i
            break;
        }

    }
    lag1(wall1-30,Y1+50);
    delay(200);
    lag1(wall1-30,Y1+250);
    delay(200);
    lag1(wall1+50,Y1+250);
    for(int i=Y1+250;i<k;i--)
    {
        lag1(wall1+50,i);////////////
        if(foot1>=1)
        {
            public:int wall2 = i
            break;
        }

    }

  }
*/



void setup() 
{
  pwm.begin();
  pwm2.begin();
  pwm.setPWMFreq(60);
  pwm2.setPWMFreq(60);
  
  Serial.begin(9600);
  Serial.print('start');
  Serial.println("SETUP");


motor0(135);
motor1(135);
motor2(135);
motor3(135);
motor4(135);
motor5(135);
motor6(135);
motor7(135);
motor8(135);
motor9(135);
motor10(135);
motor11(135);
motor12(135);
motor13(135);
motor14(135);
motor15(135);
motor16(135);
motor17(135);
delay(999999999999);

  lag0(0,h);
  lag1(0,h);
  lag2(0,h);
  lag3(0,h);
  lag4(0,h);
  lag5(0,h);
  //lag4(-100,h-20);
  //lag5(-100,h-20);

delay(5000);

  for (int i=h;i>280;i = i-3)//오른발위로
  {
    lag1(50,i);
    lag2(50,i);
    lag5(100,i); 
    delay(1);
  }
  for (int i=0;i>-50;i = i-10)//오른발앞로
  {
    lag1(i,280); 
    lag2(i,280);
    lag5(i,280);
    delay(1);
  }
  for (int i=-0;i<50;i = i+10)//왼발뒤로
  {
    lag0(i,h); 
    lag3(i,h); 
    lag4(i,h); 
    delay(3);
  }
  
  
  delay(3000);  
}


void loop() 
{

/*
lag0(0,h);
lag1(0,h);
lag2(0,h);
lag3(0,h);
lag4(0,h);
lag5(0,h);


delay(20000000);

lag0(0,h-200);
lag1(0,h-200);
lag2(0,h-200);
lag3(0,h-200);
lag4(0,h-200);
lag5(0,h-200);
delay(2000);
*/

for (int i=280;i<h;i = i+20)//오른발밑으로
  {
    lag1(-50,i); 
    lag2(-50,i);
    lag5(-50,i);
    delay(1);
  }
  for (int i=h;i>280;i = i-30)//왼발위로 
  {
    lag0(50,i);
    lag3(50,i);
    lag4(50,i); 
    delay(3);
  }
  for (int i=50;i>-50;i = i-30)//왼발앞로
  {
    lag0(i,280); 
    lag3(i,280);
    lag4(i,280);
    delay(3);
  }
  for (int i=-50;i<50;i = i+10)//오른발뒤로
  {
    lag1(i,h); 
    lag2(i,h); 
    lag5(i,h); 
    delay(1);
  }
  for (int i=280;i<h;i = i+20)//왼발밑으로
  {
    lag0(-50,i); 
    lag3(-50,i);
    lag4(-50,i);
    delay(3);
  }
  for (int i=h;i>280;i = i-30)//오른발위로
  {
    lag1(50,i);
    lag2(50,i);
    lag5(50,i); 
    delay(1);
  }
  for (int i=50;i>-50;i = i-30)//오른발앞로
  {
    lag1(i,280); 
    lag2(i,280);
    lag5(i,280);
    delay(3);
  }
for (int i=-50;i<50;i = i+10)//왼발뒤로
  {
    lag0(i,h); 
    lag3(i,h); 
    lag4(i,h); 
    delay(3);
  }


}

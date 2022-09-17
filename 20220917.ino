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


void motor0(int Angle)
{
  int a = constrain(map(Angle, 0, 270, 150, 600), 150, 600);
  pwm.setPWM(0, 0, a);
}

void motor1(int Angle)
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
  pwm.setPWM(7, 0, a);
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


void lag0(int X, int Y)
{

  float X_max = (L1 + L2);
  float X_min = (L1 - L2) * cos(PI / 4);
  float Y_max = (L1 + L2);
  float Y_min = (L1 - L2) * sin(PI / 4);

  X = constrain(X, X_min, X_max);
  Y = constrain(Y, Y_min, Y_max);

  //if (Serial.read() == '\n') {
    thetaE = acos((pow(X, 2) + pow(Y, 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2));
    thetaQ = acos((pow(X, 2) + pow(Y, 2) + pow(L1, 2) - pow(L2, 2)) / (2 * L1 * (sqrt((pow(X, 2) + pow(Y, 2))))));
    thetaS = atan2(Y, X) - thetaQ;



    //myservo.write(thetaS * 180 / PI+45);              // tell servo to go to position in variable 'pos'
    //myservo2.write(thetaE * 180 / PI+45);              // tell servo to go to position in variable 'pos'
    /*
            Serial.print(thetaE * 180 / PI);
            Serial.print(",");
            Serial.println(thetaS * 180 / PI);
    */
    float m1 = thetaS * 180 / PI + 45;
    float m2 = thetaE * 180 / PI + 45;
    motor0 (m1+90);
    motor2(180-m2);




}


void lag1(int X, int Y)
{

  float X_max = (L1 + L2);
  float X_min = (L1 - L2) * cos(PI / 4);
  float Y_max = (L1 + L2);
  float Y_min = (L1 - L2) * sin(PI / 4);

  X = constrain(X, X_min, X_max);
  Y = constrain(Y, Y_min, Y_max);

  //if (Serial.read() == '\n') {
    thetaE = acos((pow(X, 2) + pow(Y, 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2));
    thetaQ = acos((pow(X, 2) + pow(Y, 2) + pow(L1, 2) - pow(L2, 2)) / (2 * L1 * (sqrt((pow(X, 2) + pow(Y, 2))))));
    thetaS = atan2(Y, X) - thetaQ;



    //myservo.write(thetaS * 180 / PI+45);              // tell servo to go to position in variable 'pos'
    //myservo2.write(thetaE * 180 / PI+45);              // tell servo to go to position in variable 'pos'
    /*
            Serial.print(thetaE * 180 / PI);
            Serial.print(",");
            Serial.println(thetaS * 180 / PI);
    */
    float m1 = thetaS * 180 / PI + 45;
    float m2 = thetaE * 180 / PI + 45;
    motor3 (180 - m1);
    motor5(180-m2);



}




void lag2(int X, int Y)
{

  float X_max = (L1 + L2);
  float X_min = (L1 - L2) * cos(PI / 4);
  float Y_max = (L1 + L2);
  float Y_min = (L1 - L2) * sin(PI / 4);

  X = constrain(X, X_min, X_max);
  Y = constrain(Y, Y_min, Y_max);

  //if (Serial.read() == '\n') {
    thetaE = acos((pow(X, 2) + pow(Y, 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2));
    thetaQ = acos((pow(X, 2) + pow(Y, 2) + pow(L1, 2) - pow(L2, 2)) / (2 * L1 * (sqrt((pow(X, 2) + pow(Y, 2))))));
    thetaS = atan2(Y, X) - thetaQ;



    //myservo.write(thetaS * 180 / PI+45);              // tell servo to go to position in variable 'pos'
    //myservo2.write(thetaE * 180 / PI+45);              // tell servo to go to position in variable 'pos'
    /*
            Serial.print(thetaE * 180 / PI);
            Serial.print(",");
            Serial.println(thetaS * 180 / PI);
    */
    float m1 = thetaS * 180 / PI + 45;
    float m2 = thetaE * 180 / PI + 45;
    motor6 (m1+90);
    motor8(180-m2);


}



void lag3(int X, int Y)
{

  float X_max = (L1 + L2);
  float X_min = (L1 - L2) * cos(PI / 4);
  float Y_max = (L1 + L2);
  float Y_min = (L1 - L2) * sin(PI / 4);

  X = constrain(X, X_min, X_max);
  Y = constrain(Y, Y_min, Y_max);

  //if (Serial.read() == '\n') {
    thetaE = acos((pow(X, 2) + pow(Y, 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2));
    thetaQ = acos((pow(X, 2) + pow(Y, 2) + pow(L1, 2) - pow(L2, 2)) / (2 * L1 * (sqrt((pow(X, 2) + pow(Y, 2))))));
    thetaS = atan2(Y, X) - thetaQ;



    //myservo.write(thetaS * 180 / PI+45);              // tell servo to go to position in variable 'pos'
    //myservo2.write(thetaE * 180 / PI+45);              // tell servo to go to position in variable 'pos'
    /*
            Serial.print(thetaE * 180 / PI);
            Serial.print(",");
            Serial.println(thetaS * 180 / PI);
    */
    float m1 = thetaS * 180 / PI + 45;
    float m2 = thetaE * 180 / PI + 45;
    motor9 (180-m1);
    motor11(180-m2);
    Serial.print(180-m1);
            Serial.print(",");
            Serial.println(180-m2);


}



void lag4(int X, int Y)
{

  float X_max = (L1 + L2);
  float X_min = (L1 - L2) * cos(PI / 4);
  float Y_max = (L1 + L2);
  float Y_min = (L1 - L2) * sin(PI / 4);

  X = constrain(X, X_min, X_max);
  Y = constrain(Y, Y_min, Y_max);

  //if (Serial.read() == '\n') {
    thetaE = acos((pow(X, 2) + pow(Y, 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2));
    thetaQ = acos((pow(X, 2) + pow(Y, 2) + pow(L1, 2) - pow(L2, 2)) / (2 * L1 * (sqrt((pow(X, 2) + pow(Y, 2))))));
    thetaS = atan2(Y, X) - thetaQ;



    //myservo.write(thetaS * 180 / PI+45);              // tell servo to go to position in variable 'pos'
    //myservo2.write(thetaE * 180 / PI+45);              // tell servo to go to position in variable 'pos'
    /*
            Serial.print(thetaE * 180 / PI);
            Serial.print(",");
            Serial.println(thetaS * 180 / PI);
    */
    float m1 = thetaS * 180 / PI + 45;
    float m2 = thetaE * 180 / PI + 45;
    motor12 (m1+90);
    motor14(180-m2);


}


void lag5(int X, int Y)
{

  float X_max = (L1 + L2);
  float X_min = (L1 - L2) * cos(PI / 4);
  float Y_max = (L1 + L2);
  float Y_min = (L1 - L2) * sin(PI / 4);

  X = constrain(X, X_min, X_max);
  Y = constrain(Y, Y_min, Y_max);

  //if (Serial.read() == '\n') {
    thetaE = acos((pow(X, 2) + pow(Y, 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 * L2));
    thetaQ = acos((pow(X, 2) + pow(Y, 2) + pow(L1, 2) - pow(L2, 2)) / (2 * L1 * (sqrt((pow(X, 2) + pow(Y, 2))))));
    thetaS = atan2(Y, X) - thetaQ;



    //myservo.write(thetaS * 180 / PI+45);              // tell servo to go to position in variable 'pos'
    //myservo2.write(thetaE * 180 / PI+45);              // tell servo to go to position in variable 'pos'
    /*
            Serial.print(thetaE * 180 / PI);
            Serial.print(",");
            Serial.println(thetaS * 180 / PI);
    */
    float m1 = thetaS * 180 / PI + 45;
    float m2 = thetaE * 180 / PI + 45;
    motor15 (180-m1);
    motor17(180-m2);


}








  void test(){

      for (float i = 0; i < 35; i=i+1){

      Y = 35;
      X = i;
      lag1(X,Y);
delay(100);
      //delayMicroseconds(1);

    }

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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void setup() {

  pwm.begin();
  pwm2.begin();
  pwm.setPWMFreq(60);
  pwm2.setPWMFreq(60);
  
  Serial.print('start');


  Serial.begin(9600);
  Serial.println("SETUP");
  
  //pinMode(9, OUTPUT);
  //pinMode(10, OUTPUT);
  //pinMode(11, OUTPUT);
  //myservo.write(thetaS * 180 / PI+45);
}






void loop() {


//motor5(135);
    //int X1 = 350;

     







//lag0(350, 1);
//lag1(350, 1);
//lag2(350, 1);
//lag3(350, 1);
//lag4(350, 1);
//lag5(350, 1);


for(int i=1;i<300;i = i+5)
     {
        lag1(350, i);
        lag3(350, i);
        lag5(350, i);
        delay(5);
     }
     for(int i=350;i>250;i = i-10)
     {
        lag1(i, 300);
        lag3(i, 300);
        lag5(i, 300);
        delay(3);
     }
     for(int i=300;i>0;i = i-50)
     {
        lag1(250, i);
        lag3(250, i);
        lag5(250, i);
        delayMicroseconds(1);
     }
     for(int i=250;i<350;i = i+10)
     {
        lag1(i, 0);
        lag3(i, 0);
        lag5(i, 0);
        delay(3);
     }


/*
motor1(45);
    int X1 = 350;

     for(int i=1;i<350;i++)
     {
        lag0(X1, i);delay(10);
     }

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
*/
     







}

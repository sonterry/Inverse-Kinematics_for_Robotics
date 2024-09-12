#include <Servo.h>

Servo servoCoxa;  // Motor between body and l1
Servo servoFemur; // Motor between l1 and l2
Servo servoTibia; // Motor between l2 and l3

// Leg lengths in centimeters
const float l1 = 70.0;
const float l2 = 120.0;
const float l3 = 193.0;

// Pin connections
const int pinCoxa = 9;
const int pinFemur = 10;
const int pinTibia = 11;

// Initial servo positions
int angleCoxa = 90; // Base (side to side)
int angleFemur = 90; // Femur (up-down at l1)
int angleTibia = 90; // Tibia (up-down at l2)

// Pelvic coordinates of leg 3
const float pelvisX = -60.0;
const float pelvisY = 0.0;
const float pelvisZ = 0.0;

// Function to calculate angles based on target position
void calculateAngles(float targetX, float targetY, float targetZ) {
  // Transform target coordinates to leg space
  float legX = targetX - pelvisX;
  float legY = targetY - pelvisY;
  float legZ = targetZ - pelvisZ;

  // Calculate the horizontal distance in the X-Y plane
  float horizontalDistance = sqrt(legX * legX + legY * legY);

  // Coxa servo angle (side movement)
  angleCoxa = atan2(legY, legX) * 180.0 / PI;

  // Inverse kinematics for the femur and tibia
  float D = sqrt(horizontalDistance * horizontalDistance + legZ * legZ);
  float angleA = atan2(legZ, horizontalDistance) * 180.0 / PI;

  float angleB = acos((l2 * l2 + D * D - l3 * l3) / (2 * l2 * D)) * 180.0 / PI;
  angleFemur = angleA + angleB;

  float angleC = acos((l2 * l2 + l3 * l3 - D * D) / (2 * l2 * l3)) * 180.0 / PI;
  angleTibia = 180.0 - angleC;
}

void setup() {
  Serial.begin(9600);

  // Attach servos to pins
  servoCoxa.attach(pinCoxa);
  servoFemur.attach(pinFemur);
  servoTibia.attach(pinTibia);

  // Initial position
  servoCoxa.write(angleCoxa);
  servoFemur.write(angleFemur);
  servoTibia.write(angleTibia);
}

void loop() {
  // Example target coordinates for the toe (change these to move the leg)
  float targetX = 190;
  float targetY = 0;
  float targetZ = 193;

  // Calculate new angles
  calculateAngles(targetX, targetY, targetZ);

  // Move servos
  servoCoxa.write(angleCoxa);
  servoFemur.write(angleFemur);
  servoTibia.write(angleTibia);

  // Display the calculated angles
  Serial.print("Coxa Angle: ");
  Serial.println(angleCoxa);
  Serial.print("Femur Angle: ");
  Serial.println(angleFemur);
  Serial.print("Tibia Angle: ");
  Serial.println(angleTibia);

  // Wait a bit before next update
  delay(1000);
}
/*
This is a test sketch for the CA MotorShield library
Designed to work with CA MotorShield, integrating features for improved motor control.

For use with the CA MotorShield
----> http://www.yourwebsite.com/products/1234

Developed by CodingArray / hanol JU, building upon the original work by Limor Fried/Ladyada for 
Adafruit Industries and Rob Tillaart's PCA9685_RT library contributions.
*/

#include <CA_MotorShield.h>

CA_MotorShield AFMSbot(0x61); // Rightmost jumper closed
CA_MotorShield AFMStop(0x60); // Default address, no jumpers

// On the top shield, connect two steppers, each with 200 steps
CA_StepperMotor *myStepper2 = AFMStop.getStepper(200, 1);
CA_StepperMotor *myStepper3 = AFMStop.getStepper(200, 2);

// On the bottom shield connect a stepper to port M3/M4 with 200 steps
CA_StepperMotor *myStepper1 = AFMSbot.getStepper(200, 2);
// And a DC Motor to port M1
CA_DCMotor *myMotor1 = AFMSbot.getMotor(1);

void setup() {
  while (!Serial);
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Motor party!");

  AFMSbot.begin(); // Start the bottom shield
  AFMStop.begin(); // Start the top shield

  // turn on the DC motor
  myMotor1->setSpeed(200);
  myMotor1->run(RELEASE);
}

void loop() {
  myMotor1->run(FORWARD);

  for (int i = 0; i < 255; i++) {
    myMotor1->setSpeed(i);
    myStepper1->onestep(FORWARD, INTERLEAVE);
    myStepper2->onestep(BACKWARD, DOUBLE);
    myStepper3->onestep(FORWARD, MICROSTEP);
    delay(3);
 }

 for (int i = 255; i != 0; i--) {
    myMotor1->setSpeed(i);
    myStepper1->onestep(BACKWARD, INTERLEAVE);
    myStepper2->onestep(FORWARD, DOUBLE);
    myStepper3->onestep(BACKWARD, MICROSTEP);
    delay(3);
 }

  myMotor1->run(BACKWARD);

  for (int i = 0; i < 255; i++) {
    myMotor1->setSpeed(i);
    myStepper1->onestep(FORWARD, DOUBLE);
    myStepper2->onestep(BACKWARD, INTERLEAVE);
    myStepper3->onestep(FORWARD, MICROSTEP);
    delay(3);
 }

  for (int i = 255; i != 0; i--) {
    myMotor1->setSpeed(i);
    myStepper1->onestep(BACKWARD, DOUBLE);
    myStepper2->onestep(FORWARD, INTERLEAVE);
    myStepper3->onestep(BACKWARD, MICROSTEP);
    delay(3);
 }
}

/*
This is a test sketch for the CA MotorShield library
It's designed for the v2 shields with built-in PWM control.

For use with the CA MotorShield
----> http://www.yourwebsite.com/products/1234

This sketch creates a fun motor party on your desk *whiirrr*
Connect a unipolar/bipolar stepper to M3/M4
Connect a DC motor to M1
Connect a hobby servo to an available PWM pin

Developed by CodingArray / hanol JU, integrating features for improved motor control
and display capabilities, building upon the original work by Limor Fried/Ladyada for Adafruit Industries
and incorporating contributions from Rob Tillaart's PCA9685_RT library.
*/

#include <Servo.h>
#include <CA_MotorShield.h>

// Create the motor shield object with the default I2C address
CA_MotorShield AFMS = CA_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// CA_MotorShield AFMS = CA_MotorShield(0x61);

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #2 (M3 and M4)
CA_StepperMotor *myStepper = AFMS.getStepper(200, 2);
// And connect a DC motor to port M1
CA_DCMotor *myMotor = AFMS.getMotor(1);

// We'll also test out the built-in Arduino Servo library
Servo servo1;

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Motor party!");

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  // Attach a servo to an available PWM pin, e.g., pin #10
  servo1.attach(10);

  // turn on motor M1
  myMotor->setSpeed(200);
  myMotor->run(RELEASE);

  // setup the stepper
  myStepper->setSpeed(10);  // 10 rpm
}

void loop() {
  myMotor->run(FORWARD);
  for (int i = 0; i < 255; i++) {
    servo1.write(map(i, 0, 255, 0, 180));
    myMotor->setSpeed(i);
    myStepper->step(1, FORWARD, INTERLEAVE);
    delay(3);
  }

  for (int i = 255; i != 0; i--) {
    servo1.write(map(i, 0, 255, 0, 180));
    myMotor->setSpeed(i);
    myStepper->step(1, BACKWARD, INTERLEAVE);
    delay(3);
  }

  myMotor->run(BACKWARD);
  for (int i = 0; i < 255; i++) {
    servo1.write(map(i, 0, 255, 0, 180));
    myMotor->setSpeed(i);
    myStepper->step(1, FORWARD, DOUBLE);
    delay(3);
  }

  for (int i = 255; i != 0; i--) {
    servo1.write(map(i, 0, 255, 0, 180));
    myMotor->setSpeed(i);
    myStepper->step(1, BACKWARD, DOUBLE);
    delay(3);
  }
}

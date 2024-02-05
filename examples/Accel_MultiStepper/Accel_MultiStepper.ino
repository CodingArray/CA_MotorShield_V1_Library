/*!
 * @file MultipleSteppers.ino
 *
 * @mainpage Example code for running multiple steppers with CA MotorShield Library.
 *
 * @section intro_sec Introduction
 *
 * This example demonstrates how to control three stepper motors at once with
 * varying speeds using the CA_MotorShield library, an enhancement over the
 * Adafruit Motor Shield V2 library, incorporating features for improved motor control.
 *
 * Developed by CodingArray / hanol JU, building upon the original work by
 * Limor Fried/Ladyada for Adafruit Industries and Rob Tillaart's PCA9685_RT library contributions.
 *
 * This library and example are released under the MIT license.
 * Support open-source hardware by purchasing products from your preferred suppliers.
 *
 * @section license License
 *
 * MIT License - all text here must be included in any redistribution.
 *
 */

#include <CA_MotorShield.h>

// Initialize CA_MotorShield objects for two shields
CA_MotorShield AFMSbot(0x61); // Rightmost jumper closed
CA_MotorShield AFMStop(0x60); // Default address, no jumpers

// Connect two steppers to the top shield
CA_StepperMotor *myStepper1 = AFMStop.getStepper(200, 1);
CA_StepperMotor *myStepper2 = AFMStop.getStepper(200, 2);

// Connect one stepper to the bottom shield
CA_StepperMotor *myStepper3 = AFMSbot.getStepper(200, 2);

// Function wrappers for each motor
void forwardStep1() { myStepper1->onestep(FORWARD, SINGLE); }
void backwardStep1() { myStepper1->onestep(BACKWARD, SINGLE); }
void forwardStep2() { myStepper2->onestep(FORWARD, DOUBLE); }
void backwardStep2() { myStepper2->onestep(BACKWARD, DOUBLE); }
void forwardStep3() { myStepper3->onestep(FORWARD, INTERLEAVE); }
void backwardStep3() { myStepper3->onestep(BACKWARD, INTERLEAVE); }

// Wrap the steppers in CA_Stepper objects
CA_Stepper stepper1(forwardStep1, backwardStep1);
CA_Stepper stepper2(forwardStep2, backwardStep2);
CA_Stepper stepper3(forwardStep3, backwardStep3);

void setup() {
  AFMSbot.begin(); // Start the bottom shield
  AFMStop.begin(); // Start the top shield

  // Set speed and acceleration for each stepper
  stepper1.setMaxSpeed(100.0);
  stepper1.setAcceleration(100.0);
  stepper1.moveTo(24);

  stepper2.setMaxSpeed(200.0);
  stepper2.setAcceleration(100.0);
  stepper2.moveTo(50000);

  stepper3.setMaxSpeed(300.0);
  stepper3.setAcceleration(100.0);
  stepper3.moveTo(1000000);
}

void loop() {
  // Change direction at the limits
  if (stepper1.distanceToGo() == 0) stepper1.moveTo(-stepper1.currentPosition());
  if (stepper2.distanceToGo() == 0) stepper2.moveTo(-stepper2.currentPosition());
  if (stepper3.distanceToGo() == 0) stepper3.moveTo(-stepper3.currentPosition());

  // Run each stepper
  stepper1.run();
  stepper2.run();
  stepper3.run();
}

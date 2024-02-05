/*!
 * @file ConstantSpeed.ino
 *
 * @mainpage Example code for CA MotorShield Library.
 *
 * @section intro_sec Introduction
 *
 * This example shows how to control a stepper motor using the CA_MotorShield library,
 * demonstrating the simplest, fixed speed mode with no accelerations.
 * It is designed for the CA MotorShield, which is an enhancement over the Adafruit Motor Shield V2,
 * integrating features for improved motor control.
 *
 * Developed by CodingArray / hanol JU, based on the original work by
 * Limor Fried/Ladyada for Adafruit Industries and enhancements from Rob Tillaart's PCA9685_RT library.
 *
 * This library is released under the MIT license.
 * Please support CA MotorShield and open-source hardware by purchasing
 * products from CodingArray / www.codingarray.net.
 *
 * @section license License
 *
 * MIT License - all text here must be included in any redistribution.
 *
 */

#include <CA_MotorShield.h>

// Create the motor shield object with the default I2C address
CA_MotorShield STEP_Motor(0x60); // Adjust the I2C address as needed

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #1 and #2 respectively
CA_StepperMotor *StepMotor1 = STEP_Motor.getStepper(200, 1);

void forwardStep() {
  StepMotor1->onestep(FORWARD, SINGLE);
}
void backwardStep() {
  StepMotor1->onestep(BACKWARD, SINGLE);
}

// Use functions to step
CA_Stepper Astepper1 = CA_Stepper(forwardStep, backwardStep);

void setup() {
  Serial.begin(115200); // set up Serial library at 115200 bps
  Serial.println("Stepper test!");

  STEP_Motor.begin(); // Initialize the motor shield
  STEP_Motor.setFrequency(1600); // Set PWM frequency for stepper motor

  Astepper1.setSpeed(150); // Set the speed of the stepper motor
}

void loop() {
  Astepper1.runSpeed(); // Continuously run the stepper motor at the set speed
}

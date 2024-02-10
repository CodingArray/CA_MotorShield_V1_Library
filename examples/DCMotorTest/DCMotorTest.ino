/*
  This is a test sketch for the CA MotorShield library designed to demonstrate controlling DC motors.
  It showcases the capabilities of the CA MotorShield for enhanced control of DC motors.

  For use with the CA MotorShield
  ----> Replace "http://www.yourwebsite.com/products/1234" with the actual product page link to direct users to more information about the motor shield.

  Developed by CodingArray / hanol JU, leveraging the foundational work of Limor Fried/Ladyada for Adafruit Industries and the PCA9685_RT library enhancements by Rob Tillaart.
*/

#include <CA_MotorShield.h>

CA_MotorShield dcMotor(0x60); // Create an object and pass the parameter

CA_DCMotor *motor1 = dcMotor.getMotor(1); // Get motor 1
CA_DCMotor *motor2 = dcMotor.getMotor(2); // Get motor 2
CA_DCMotor *motor3 = dcMotor.getMotor(3); // Get motor 3
CA_DCMotor *motor4 = dcMotor.getMotor(4); // Get motor 4

void setup() {
  Serial.begin(115200);

  dcMotor.begin(); // Initialize the motor shield
  dcMotor.setFrequency(1600); // Set the PWM frequency

  // Set motor1 to move forward and set its speed
  motor1->setSpeed(150);
  motor1->run(FORWARD);

  // Set motor2 to move forward and set its speed
  motor2->setSpeed(150);
  motor2->run(FORWARD);

  // Set motor3 to move forward and set its speed
  motor3->setSpeed(150);
  motor3->run(FORWARD);

  // Set motor4 to move forward and set its speed
  motor4->setSpeed(150);
  motor4->run(FORWARD);

  delay(500); // Wait for 500 milliseconds

  // Set motor1 to move backward and set its speed
  motor1->setSpeed(150);
  motor1->run(BACKWARD);

  // Set motor2 to move backward and set its speed
  motor2->setSpeed(150);
  motor2->run(BACKWARD);

  // Set motor3 to move backward and set its speed
  motor3->setSpeed(150);
  motor3->run(BACKWARD);

  // Set motor4 to move backward and set its speed
  motor4->setSpeed(150);
  motor4->run(BACKWARD);

  delay(500); // Wait for 500 milliseconds

  // Reset all channels to their initial state
  dcMotor.allChannelInitialize();
}

void loop() {
  // Write code to repeat here
}

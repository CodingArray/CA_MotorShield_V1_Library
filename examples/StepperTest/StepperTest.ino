/*
  This is a test sketch for the CA MotorShield library
  Designed to work with CA MotorShield, integrating features for improved motor control.

  For use with the CA MotorShield
  ----> Replace "http://www.yourwebsite.com/products/1234" with the actual product page link

  Developed by CodingArray / hanol JU, building upon the original work by Limor Fried/Ladyada for 
  Adafruit Industries and Rob Tillaart's PCA9685_RT library contributions.
*/

#include <CA_MotorShield.h> // Include library

// Create MotorShield object. Pass I2C address as a parameter.
CA_MotorShield STEP_Motor(0x60);

// Create stepper motor objects. Use the getStepper function for each stepper motor configuration.
// The first parameter of getStepper function is the number of steps, and the second parameter is the motor number.
CA_StepperMotor *StepMotor1 = STEP_Motor.getStepper(200, 1); // First stepper motor
CA_StepperMotor *StepMotor2 = STEP_Motor.getStepper(200, 2); // Second stepper motor

void setup() {
  Serial.begin(115200); // Start serial communication. For debugging information output.

  // Initialize motor shield and set PWM frequency
  STEP_Motor.begin(); // Initialize motor shield
  STEP_Motor.setFrequency(1600); // Set PWM frequency

  // Set speed for each stepper motor
  StepMotor1->setSpeed(150); // Set speed for the first motor
  StepMotor2->setSpeed(150); // Set speed for the second motor

  // Test step sequences for the first stepper motor
  Serial.println("Single coil steps");
  StepMotor1->step(100, FORWARD, SINGLE); // Single coil step
  StepMotor1->step(100, BACKWARD, SINGLE);

  Serial.println("Double coil steps");
  StepMotor1->step(100, FORWARD, DOUBLE); // Double coil step
  StepMotor1->step(100, BACKWARD, DOUBLE);

  Serial.println("Interleave coil steps");
  StepMotor1->step(100, FORWARD, INTERLEAVE); // Interleave coil step
  StepMotor1->step(100, BACKWARD, INTERLEAVE);

  Serial.println("Microstep steps");
  StepMotor1->step(50, FORWARD, MICROSTEP); // Microstep step
  StepMotor1->step(50, BACKWARD, MICROSTEP);
  delay(500); // Short delay before executing next command

  // Test step sequences for the second stepper motor
  StepMotor2->step(100, FORWARD, SINGLE); // Single coil step
  StepMotor2->step(100, BACKWARD, SINGLE);

  Serial.println("Double coil steps");
  StepMotor2->step(100, FORWARD, DOUBLE); // Double coil step
  StepMotor2->step(100, BACKWARD, DOUBLE);

  Serial.println("Interleave coil steps");
  StepMotor2->step(100, FORWARD, INTERLEAVE); // Interleave coil step
  StepMotor2->step(100, BACKWARD, INTERLEAVE);

  Serial.println("Microstep steps");
  StepMotor2->step(50, FORWARD, MICROSTEP); // Microstep step
  StepMotor2->step(50, BACKWARD, MICROSTEP);
  delay(500); // Short delay

  // Call function to initialize all channels
  STEP_Motor.allChannelInitialize();
}

void loop() {
  // The loop() function is empty in this example. Implement repeating logic here in actual use.
}

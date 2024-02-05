/*!
 * @file MotorSpeedEncoderDisplay.ino
 *
 * @mainpage Example code for CA MotorShield with RPM measurement and OLED display.
 *
 * @section intro_sec Introduction
 *
 * This example demonstrates how to measure the speed and direction of a DC motor
 * using the CA_MotorShield library, and display the results on an OLED screen.
 * It utilizes a motor encoder for RPM measurement.
 *
 * Developed by CodingArray / hanol JU, this project integrates features for improved 
 * motor control and display capabilities, building upon the original work by Limor 
 * Fried/Ladyada for Adafruit Industries and incorporating contributions from Rob Tillaart's 
 * PCA9685_RT library.
 *
 * This library and example are released under the MIT license.
 * Support open-source hardware by purchasing products from your preferred suppliers.
 *
 * @section license License
 *
 * MIT License - all text here must be included in any redistribution.
 *
 */

#include <Wire.h>
#include <CA_MotorShield.h>
#include <Adafruit_SSD1306.h>

#define ENCODER_A   12
#define ENCODER_B   11
#define GEARING     20
#define ENCODERMULT 12

CA_MotorShield DC_Motor(0x60); // Adjust the I2C address as needed
CA_DCMotor *myMotor = DC_Motor.getMotor(1);
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

volatile float RPM = 0;
volatile uint32_t lastA = 0;
volatile bool motordir = FORWARD;

void interruptA() {
  motordir = digitalRead(ENCODER_B);
  digitalWrite(LED_BUILTIN, HIGH);
  uint32_t currA = micros();
  if (lastA < currA) {
    float rev = currA - lastA;
    rev = 1.0 / rev;
    rev *= 1000000;
    rev *= 60;
    rev /= GEARING;
    rev /= ENCODERMULT;
    RPM = rev;
  }
  lastA = currA;
  digitalWrite(LED_BUILTIN, LOW);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(ENCODER_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), interruptA, RISING);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.display();
  display.setTextSize(2);
  display.setTextColor(WHITE);

  Serial.println("Motor Encoder Test");
  DC_Motor.begin();
  DC_Motor.setFrequency(1600);

  myMotor->setSpeed(0);
}

void printRPM() {
    display.clearDisplay();
    display.setCursor(0,0);

    Serial.print("Direction: ");
    if (motordir) {
      display.println("Forward");
      Serial.println("forward @ ");
    } else {
      display.println("Backward");
      Serial.println("backward @ ");
    }
    display.print((int)RPM); display.println(" RPM");
    Serial.print((int)RPM); Serial.println(" RPM");
    display.display();
}

void loop() {
  delay(50);
  myMotor->run(FORWARD);
  for (int i = 0; i < 255; i++) {
    myMotor->setSpeed(i);
    delay(20);
    printRPM();
  }

  for (int i = 255; i != 0; i--) {
    myMotor->setSpeed(i);
    delay(20);
    printRPM();
  }

  myMotor->run(BACKWARD);
  for (int i = 0; i < 255; i++) {
    myMotor->setSpeed(i);
    delay(20);
    printRPM();
  }

  for (int i = 255; i != 0; i--) {
    myMotor->setSpeed(i);
    delay(20);
    printRPM();
  }
}

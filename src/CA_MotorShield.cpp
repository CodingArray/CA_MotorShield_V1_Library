/*!
 * @file CA_MotorShield.cpp
 *
 * @mainpage CA MotorShield Library
 *
 * @section intro_sec Introduction
 *
 * This is the library for the CA MotorShield, an enhanced library 
 * combining features from Adafruit Motor Shield V2 and PCA9685_RT 
 * library to address specific issues with DC motor control and 
 * I2C communication errors. It supports DC motors, Stepper motors 
 * with microstepping, and offers stacking-support.
 * 
 * This library is designed for use with the Adafruit Motor Shield 
 * https://www.adafruit.com/products/1483 and Motor FeatherWing 
 * https://www.adafruit.com/product/2927, integrating improvements 
 * for reliable motor direction switching and precise PWM control 
 * via the PCA9685 module.
 *
 * It uses I2C to communicate, requiring 2 pins (SCL+SDA) to interface.
 *
 * Development of this library was driven by the need to resolve 
 * multi-pin HIGH/LOW setting issues in 'void Adafruit_MotorShield::setPin(uint8_t pin, boolean value)'
 * function and to enhance motor control capabilities.
 *
 * @section author Author
 *
 * Developed by CodingArray / hanol JU, based on original work by 
 * Limor Fried/Ladyada for Adafruit Industries and Rob Tillaart.
 *
 * @section license License
 *
 * This library is released under the MIT license, reflecting the 
 * integration with the PCA9685_RT library and enhancements made.
 * Original Adafruit MotorShield V2 library is under BSD license.
 * All text from the original libraries must be included in any redistribution.
 * 
 * URL: https://github.com/CodingArray/CA_MotorShield_V1_Library
 */

#include "CA_MotorShield.h"

//////////////////////////////////////////////////////////////
//
//  microstepcurve
//
#if (MICROSTEPS == 8)
///! A sinusoial microstepping curve for the PWM output (8-bit range) with 9
/// points - last one is start of next step.
static uint8_t microstepcurve[] = {0, 50, 98, 142, 180, 212, 236, 250, 255};
#elif (MICROSTEPS == 16)
///! A sinusoial microstepping curve for the PWM output (8-bit range) with 17
/// points - last one is start of next step.
static uint8_t microstepcurve[] = {0,   25,  50,  74,  98,  120, 141, 162, 180,
                                   197, 212, 225, 236, 244, 250, 253, 255};
#endif

//////////////////////////////////////////////////////////////
//
//  Constructor
//
// Constructor: Initializes the motor shield with a specific I2C address and wire interface
CA_MotorShield::CA_MotorShield(const uint8_t deviceAddress, TwoWire *wire) : _address(deviceAddress), _wire(wire) {
  _wire->begin(); // Start I2C communication
}

// Default begin function: Initializes the motor shield with default settings
bool CA_MotorShield::begin() {
  if (!isConnected()) return false; // Check if shield is connected
  uint8_t mode1_mask = PCA9685_MODE1_AUTOINCR | PCA9685_MODE1_ALLCALL; // Default mode1 settings
  uint8_t mode2_mask = PCA9685_MODE2_TOTEMPOLE; // Default mode2 settings
  configure(mode1_mask, mode2_mask); // Apply configuration settings
  allChannelInitialize(); // Initialize all PWM channels
  setFrequency(_freq); // Set to default frequency
  return true; // Initialization successful
}

// Overloaded begin function: Initializes the motor shield with a specified PWM frequency
bool CA_MotorShield::begin(uint16_t pwmFrequency) {
  if (!isConnected()) return false; // Check if shield is connected
  _freq = pwmFrequency; // Set the specified frequency
  uint8_t mode1_mask = PCA9685_MODE1_AUTOINCR | PCA9685_MODE1_ALLCALL; // Default mode1 settings
  uint8_t mode2_mask = PCA9685_MODE2_TOTEMPOLE; // Default mode2 settings
  configure(mode1_mask, mode2_mask); // Apply configuration settings
  allChannelInitialize(); // Initialize all PWM channels
  setFrequency(_freq); // Set the PWM frequency as specified
  return true; // Initialization successful
}

uint8_t CA_MotorShield::configure(uint8_t mode1_mask, uint8_t mode2_mask)
{
  _error = PCA9685_OK;

  uint8_t r1 = setMode1(mode1_mask);
  uint8_t r2 = setMode2(mode2_mask);

  if ((r1 != PCA9685_OK) || (r2 != PCA9685_OK))
  {
    return PCA9685_ERROR;
  }
  return _error;
}

bool CA_MotorShield::isConnected()
{
  _wire->beginTransmission(_address);
  _error = _wire->endTransmission();
  return (_error == 0);
}

uint8_t CA_MotorShield::getAddress()
{
  return _address;
}

uint8_t CA_MotorShield::channelCount()
{
  return _channelCount;
}

uint8_t CA_MotorShield::writeMode(uint8_t reg, uint8_t value)
{
  if ((reg == PCA9685_MODE1) || (reg == PCA9685_MODE2))
  {
    writeReg(reg, value);
    return PCA9685_OK;
  }
  _error = PCA9685_ERR_MODE;
  return PCA9685_ERROR;
}

uint8_t CA_MotorShield::readMode(uint8_t reg)
{
  if ((reg == PCA9685_MODE1) || (reg == PCA9685_MODE2))
  {
    _error = PCA9685_OK;
    uint8_t value = readReg(reg);
    return value;
  }
  _error = PCA9685_ERR_MODE;
  return PCA9685_ERROR;
}

uint8_t CA_MotorShield::setMode1(uint8_t value)
{
  return writeMode(PCA9685_MODE1, value);
}

uint8_t CA_MotorShield::setMode2(uint8_t value)
{
  return writeMode(PCA9685_MODE2, value);
}


uint8_t CA_MotorShield::getMode1()
{
  return readMode(PCA9685_MODE1);
}


uint8_t CA_MotorShield::getMode2()
{
  return readMode(PCA9685_MODE2);
}

//  write value to single PWM channel
void CA_MotorShield::setPWM(uint8_t channel, uint16_t onTime, uint16_t offTime)
{
  _error = PCA9685_OK;
  if (channel >= _channelCount)
  {
    _error = PCA9685_ERR_CHANNEL;
    return;
  }
  offTime &= 0x0FFFF;   // non-doc feature - to easy set figure 8 P.17
  uint8_t reg = PCA9685_CHANNEL(channel);
  writeReg2(reg, onTime, offTime);
}

//  write value to single PWM channel
void CA_MotorShield::setPWM(uint8_t channel, uint16_t offTime)
{
  setPWM(channel, 0, offTime);
}

void CA_MotorShield::allChannelInitialize()
{
  for (int channel = 0; channel < channelCount(); channel++)
  {
    setPWM(channel, 0, 1000);
  }
  allOFF();
}

//  read value from single PWM channel
void CA_MotorShield::getPWM(uint8_t channel, uint16_t* onTime, uint16_t* offTime)
{
  _error = PCA9685_OK;
  if (channel >= _channelCount)
  {
    _error = PCA9685_ERR_CHANNEL;
    return;
  }
  uint8_t reg = PCA9685_CHANNEL(channel);
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _error = _wire->endTransmission();
  if (_wire->requestFrom(_address, (uint8_t)4) != 4)
  {
    _error = PCA9685_ERR_I2C;
    return;
  }
  uint16_t _data = _wire->read();
  *onTime = (_wire->read() * 256) + _data;
  _data = _wire->read();
  *offTime = (_wire->read() * 256) + _data;
}

//  set update frequency for all channels
void CA_MotorShield::setFrequency(uint16_t freq, int offset)
{
  _error = PCA9685_OK;
  _freq = freq;
  //  limits frequency see page 25 datasheet
  if (_freq < PCA9685_MIN_FREQ) _freq = PCA9685_MIN_FREQ;
  if (_freq > PCA9685_MAX_FREQ) _freq = PCA9685_MAX_FREQ;
  //  removed float operation for speed
  //  faster and equal accurate
  //  uint8_t scaler = round(25e6 / (_freq * 4096)) - 1;
  uint8_t scaler = 48828 / (_freq * 8) - 1;

  uint8_t mode1 = readMode(PCA9685_MODE1);
  writeMode(PCA9685_MODE1, mode1 | PCA9685_MODE1_SLEEP);
  scaler += offset;
  writeReg(PCA9685_PRE_SCALER, scaler);
  writeMode(PCA9685_MODE1, mode1);
}

//  returns the actual used frequency.
//  therefore it does not use offset
int CA_MotorShield::getFrequency(bool cache)
{
  _error = PCA9685_OK;
  if (cache) return _freq;
  uint8_t scaler = readReg(PCA9685_PRE_SCALER);
  scaler++;
  _freq = 48828 / scaler;
  _freq /= 8;
  return _freq;
}

//  datasheet P.18 - fig. 9:
//  Note: bit[11-0] ON should NOT equal timer OFF in ON mode
//  in OFF mode it doesn't matter.
void CA_MotorShield::write1(uint8_t channel, uint8_t mode)
{
  _error = PCA9685_OK;
  if (channel >= _channelCount)
  {
    _error = PCA9685_ERR_CHANNEL;
    return;
  }
  uint8_t reg = PCA9685_CHANNEL(channel);
  if (mode != LOW) writeReg2(reg, 0x1000, 0x0000);
  else writeReg2(reg, 0x0000, 0x0000);
}

void CA_MotorShield::allOFF()
{
  _error = PCA9685_OK;
  writeReg(PCA9685_ALL_OFF_H, 0x10);
}

int CA_MotorShield::lastError()
{
  int e = _error;
  _error = PCA9685_OK;
  return e;
}

/////////////////////////////////////////////////////
//
//  SUB CALL  -   ALL CALL
//
bool CA_MotorShield::enableSubCall(uint8_t nr)
{
  if ((nr == 0) || (nr > 3)) return false;
  uint8_t prev = getMode1();
  uint8_t reg = prev;
  if (nr == 1)      reg |= PCA9685_MODE1_SUB1;
  else if (nr == 2) reg |= PCA9685_MODE1_SUB2;
  else              reg |= PCA9685_MODE1_SUB3;
  //  only update if changed.
  if (reg != prev) setMode1(reg);
  return true;
}

bool CA_MotorShield::disableSubCall(uint8_t nr)
{
  if ((nr == 0) || (nr > 3)) return false;
  uint8_t prev = getMode1();
  uint8_t reg = prev;
  if (nr == 1)      reg &= ~PCA9685_MODE1_SUB1;
  else if (nr == 2) reg &= ~PCA9685_MODE1_SUB2;
  else              reg &= ~PCA9685_MODE1_SUB3;
  //  only update if changed.
  if (reg != prev) setMode1(reg);
  return true;
}

bool CA_MotorShield::isEnabledSubCall(uint8_t nr)
{
  if ((nr == 0) || (nr > 3)) return false;
  uint8_t reg = getMode1();
  if (nr == 1) return (reg & PCA9685_MODE1_SUB1) > 0;
  if (nr == 2) return (reg & PCA9685_MODE1_SUB2) > 0;
  return (reg & PCA9685_MODE1_SUB3) > 0;
}

bool CA_MotorShield::setSubCallAddress(uint8_t nr, uint8_t address)
{
  if ((nr == 0) || (nr > 3)) return false;
  writeReg(PCA9685_SUBADR(nr), address);
  return true;
}

uint8_t CA_MotorShield::getSubCallAddress(uint8_t nr)
{
  if ((nr == 0) || (nr > 3)) return 0;
  uint8_t address = readReg(PCA9685_SUBADR(nr));
  return address;
}

bool CA_MotorShield::enableAllCall()
{
  uint8_t prev = getMode1();
  uint8_t reg = prev | PCA9685_MODE1_ALLCALL;
  //  only update if changed.
  if (reg != prev) setMode1(reg);
  return true;
}

bool CA_MotorShield::disableAllCall()
{
  uint8_t prev = getMode1();
  uint8_t reg = prev & ~PCA9685_MODE1_ALLCALL;
  //  only update if changed.
  if (reg != prev) setMode1(reg);
  return true;
}

bool CA_MotorShield::isEnabledAllCall()
{
  uint8_t reg = getMode1();
  return reg & PCA9685_MODE1_ALLCALL;
}

bool CA_MotorShield::setAllCallAddress(uint8_t address)
{
  writeReg(PCA9685_ALLCALLADR, address);
  return true;
}

uint8_t CA_MotorShield::getAllCallAddress()
{
  uint8_t address = readReg(PCA9685_ALLCALLADR);
  return address;
}

/////////////////////////////////////////////////////
//
//  OE - Output Enable control
//
//  active LOW see datasheet
//
bool CA_MotorShield::setOutputEnablePin(uint8_t pin)
{
  _OutputEnablePin = pin;
  if (_OutputEnablePin != 255)
  {
    pinMode(_OutputEnablePin, OUTPUT);
    write1(_OutputEnablePin, HIGH);
    return true;
  }
  //  must it be set to HIGH now?
  return false;
}

bool CA_MotorShield::setOutputEnable(bool on)
{
  if (_OutputEnablePin != 255)
  {
    write1(_OutputEnablePin, on ? LOW : HIGH);
    return true;
  }
  return false;
}

uint8_t CA_MotorShield::getOutputEnable()
{
  if (_OutputEnablePin != 255)
  {
    return digitalRead(_OutputEnablePin);
  }
  return HIGH;
}

//////////////////////////////////////////////////////
//
//  EXPERIMENTAL
//
//Method 1 is a tested method which is specific to the PCA9634. Since the number of different types of I²C chips is very large, side-effects on other chips might be possible. Before using this method, consult the data sheets of all chips on the bus to mitigate potential undefined states.
//Method 0 is a somewhat “general” method which resets many chips on the I²C-bus. However, this method DOES NOT reset the PCA9634 chip. Therefore, consult the data sheet of all different chips on the bus to mitigate potential undefined states.
int CA_MotorShield::I2C_SoftwareReset(uint8_t method)
{
  //  only support 0 and 1
  if (method > 1) return -999;
  if (method == 1)
  {
    //  from https://github.com/RobTillaart/PCA9634/issues/10#issuecomment-1206326417
   const uint8_t SW_RESET = 0x03;
   _wire->beginTransmission(SW_RESET);
   _wire->write(0xA5);
   _wire->write(0x5A);
   return _wire->endTransmission(true);
  }

  //  default - based upon NXP specification - UM10204.pdf - page 16
  _wire->beginTransmission(0x00);
  _wire->write(0x06);
  return _wire->endTransmission(true);
}

//////////////////////////////////////////////////////////////
//
//  PRIVATE
//
void CA_MotorShield::writeReg(uint8_t reg, uint8_t value)
{
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->write(value);
  _error = _wire->endTransmission();
}

void CA_MotorShield::writeReg2(uint8_t reg, uint16_t a, uint16_t b)
{
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->write(a & 0xFF);
  _wire->write((a >> 8) & 0x1F);
  _wire->write(b & 0xFF);
  _wire->write((b >> 8) & 0x1F);
  _error = _wire->endTransmission();
}

uint8_t CA_MotorShield::readReg(uint8_t reg)
{
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _error = _wire->endTransmission();
  if (_wire->requestFrom(_address, (uint8_t)1) != 1)
  {
    _error = PCA9685_ERR_I2C;
    return 0;
  }
  uint8_t _data = _wire->read();
  return _data;
}

// -- END OF FILE --
void CA_MotorShield::setPin(uint8_t pin, boolean value)
{
    write1(pin, value);
}

CA_DCMotor *CA_MotorShield::getMotor(uint8_t num) {
  if (num > 4)
    return NULL;

  num--;

  if (dcmotors[num].motornum == 0) {
    // not init'd yet!
    dcmotors[num].motornum = num;
    dcmotors[num].MC = this;
    uint8_t pwm, in1, in2;
    switch (num) {
    case 0:
      pwm = 8;
      in2 = 9;
      in1 = 10;
      break;
    case 1:
      pwm = 13;
      in2 = 12;
      in1 = 11;
      break;
    case 2:
      pwm = 2;
      in2 = 3;
      in1 = 4;
      break;
    default:
      pwm = 7;
      in2 = 6;
      in1 = 5;
      break;
    }
    dcmotors[num].PWMpin = pwm;
    dcmotors[num].IN1pin = in1;
    dcmotors[num].IN2pin = in2;
  }
  return &dcmotors[num];
}

CA_StepperMotor *CA_MotorShield::getStepper(uint16_t steps, uint8_t num) {
  if (num > 2)
    return NULL;

  num--;

  if (steppers[num].steppernum == 0) {
    // not init'd yet!
    steppers[num].steppernum = num;
    steppers[num].revsteps = steps;
    steppers[num].MC = this;
    uint8_t pwma, pwmb, ain1, ain2, bin1, bin2;
    if (num == 0) {
      pwma = 8;
      ain2 = 9;
      ain1 = 10;
      pwmb = 13;
      bin2 = 12;
      bin1 = 11;
    } else {
      pwma = 2;
      ain2 = 3;
      ain1 = 4;
      pwmb = 7;
      bin2 = 6;
      bin1 = 5;
    }
    steppers[num].PWMApin = pwma;
    steppers[num].PWMBpin = pwmb;
    steppers[num].AIN1pin = ain1;
    steppers[num].AIN2pin = ain2;
    steppers[num].BIN1pin = bin1;
    steppers[num].BIN2pin = bin2;
  }
  return &steppers[num];
}

/******************************************
               MOTORS
******************************************/
CA_DCMotor::CA_DCMotor(void) {
  MC = NULL;
  motornum = 0;
  PWMpin = IN1pin = IN2pin = 0;
}

// Controls the motor's direction of rotation
void CA_DCMotor::run(uint8_t cmd) {
  switch (cmd) {
    case FORWARD:
      //모터 방향 전환 간 급격한 전류 증가를 방지함
      MC->setPin(IN2pin, LOW); // Set both control pins low
      MC->setPin(IN1pin, LOW); // to stop the motor
      delay(5); // Short delay to ensure the setup is stable
      MC->setPin(IN2pin, LOW); // FORWARD 핀 전환
      MC->setPin(IN1pin, HIGH); 
      break;
    case BACKWARD:
      //모터 방향 전환 간 급격한 전류 증가를 방지함
      MC->setPin(IN1pin, LOW); // Set both control pins low
      MC->setPin(IN2pin, LOW); // to stop the motor
      delay(5); // Short delay to ensure the setup is stable
      MC->setPin(IN1pin, LOW); // BACKWARD 핀 전환
      MC->setPin(IN2pin, HIGH); 
      break;
    case RELEASE:
      // Stop the motor
      MC->setPin(IN1pin, LOW); // Set both control pins low
      MC->setPin(IN2pin, LOW); // to stop the motor
      break;
  }
}

// Sets the motor's speed
void CA_DCMotor::setSpeed(uint16_t speed) {
  // Use the Arduino map function to convert the speed value from 0-100 to 0-2400
  uint16_t pwmValue = map(speed, 0, 100, 0, 2400);
  // Set the PWM value to control the motor speed
  MC->setPWM(PWMpin, pwmValue);
}

// Sets the motor's speed without a limit, mapping from a 0-255 range to a 0-4095 range using the Arduino map function
void CA_DCMotor::setSpeedNoLimit(uint16_t speed) {
  // Use the Arduino map function to map the speed value from 0-255 to 0-4095
  uint16_t pwmValue = map(speed, 0, 255, 0, 4095);

  // Set the PWM value to control the motor speed
  MC->setPWM(PWMpin, pwmValue);
}

void CA_DCMotor::setSpeedFine(uint16_t speed) {
  MC->setPWM(PWMpin, speed > 4095 ? 4095 : speed);
}

/******************************************
               STEPPERS
******************************************/
CA_StepperMotor::CA_StepperMotor(void) {
  revsteps = steppernum = currentstep = 0;
}

void CA_StepperMotor::setSpeed(uint16_t rpm) {
  // Serial.println("steps per rev: "); Serial.println(revsteps);
  // Serial.println("RPM: "); Serial.println(rpm);

  usperstep = 60000000 / ((uint32_t)revsteps * (uint32_t)rpm);
}

void CA_StepperMotor::release(void) {
  MC->setPin(AIN1pin, LOW);
  MC->setPin(AIN2pin, LOW);
  MC->setPin(BIN1pin, LOW);
  MC->setPin(BIN2pin, LOW);
  MC->setPWM(PWMApin, 0);
  MC->setPWM(PWMBpin, 0);
}

void CA_StepperMotor::step(uint16_t steps, uint8_t dir, uint8_t style) {
  uint32_t uspers = usperstep;

  if (style == INTERLEAVE) {
    uspers /= 2;
  } else if (style == MICROSTEP) {
    uspers /= MICROSTEPS;
    steps *= MICROSTEPS;
#ifdef MOTORDEBUG
    Serial.print("steps = ");
    Serial.println(steps, DEC);
#endif
  }

  while (steps--) {
    // Serial.println("step!"); Serial.println(uspers);
    onestep(dir, style);
    delayMicroseconds(uspers);
#ifdef ESP8266
    yield(); // required for ESP8266
#endif
  }
}

uint8_t CA_StepperMotor::onestep(uint8_t dir, uint8_t style) {
  uint8_t ocrb, ocra;

  ocra = ocrb = 255;

  // next determine what sort of stepping procedure we're up to
  if (style == SINGLE) {
    if ((currentstep / (MICROSTEPS / 2)) % 2) { // we're at an odd step, weird
      if (dir == FORWARD) {
        currentstep += MICROSTEPS / 2;
      } else {
        currentstep -= MICROSTEPS / 2;
      }
    } else { // go to the next even step
      if (dir == FORWARD) {
        currentstep += MICROSTEPS;
      } else {
        currentstep -= MICROSTEPS;
      }
    }
  } else if (style == DOUBLE) {
    if (!(currentstep / (MICROSTEPS / 2) % 2)) { // we're at an even step, weird
      if (dir == FORWARD) {
        currentstep += MICROSTEPS / 2;
      } else {
        currentstep -= MICROSTEPS / 2;
      }
    } else { // go to the next odd step
      if (dir == FORWARD) {
        currentstep += MICROSTEPS;
      } else {
        currentstep -= MICROSTEPS;
      }
    }
  } else if (style == INTERLEAVE) {
    if (dir == FORWARD) {
      currentstep += MICROSTEPS / 2;
    } else {
      currentstep -= MICROSTEPS / 2;
    }
  }

  if (style == MICROSTEP) {
    if (dir == FORWARD) {
      currentstep++;
    } else {
      // BACKWARDS
      currentstep--;
    }

    currentstep += MICROSTEPS * 4;
    currentstep %= MICROSTEPS * 4;

    ocra = ocrb = 0;
    if (currentstep < MICROSTEPS) {
      ocra = microstepcurve[MICROSTEPS - currentstep];
      ocrb = microstepcurve[currentstep];
    } else if ((currentstep >= MICROSTEPS) && (currentstep < MICROSTEPS * 2)) {
      ocra = microstepcurve[currentstep - MICROSTEPS];
      ocrb = microstepcurve[MICROSTEPS * 2 - currentstep];
    } else if ((currentstep >= MICROSTEPS * 2) &&
               (currentstep < MICROSTEPS * 3)) {
      ocra = microstepcurve[MICROSTEPS * 3 - currentstep];
      ocrb = microstepcurve[currentstep - MICROSTEPS * 2];
    } else if ((currentstep >= MICROSTEPS * 3) &&
               (currentstep < MICROSTEPS * 4)) {
      ocra = microstepcurve[currentstep - MICROSTEPS * 3];
      ocrb = microstepcurve[MICROSTEPS * 4 - currentstep];
    }
  }

  currentstep += MICROSTEPS * 4;
  currentstep %= MICROSTEPS * 4;

#ifdef MOTORDEBUG
  Serial.print("current step: ");
  Serial.println(currentstep, DEC);
  Serial.print(" pwmA = ");
  Serial.print(ocra, DEC);
  Serial.print(" pwmB = ");
  Serial.println(ocrb, DEC);
#endif
  MC->setPWM(PWMApin, ocra * 16);
  MC->setPWM(PWMBpin, ocrb * 16);

  // release all
  uint8_t latch_state = 0; // all motor pins to 0

  // Serial.println(step, DEC);
  if (style == MICROSTEP) {
    if (currentstep < MICROSTEPS)
      latch_state |= 0x03;
    if ((currentstep >= MICROSTEPS) && (currentstep < MICROSTEPS * 2))
      latch_state |= 0x06;
    if ((currentstep >= MICROSTEPS * 2) && (currentstep < MICROSTEPS * 3))
      latch_state |= 0x0C;
    if ((currentstep >= MICROSTEPS * 3) && (currentstep < MICROSTEPS * 4))
      latch_state |= 0x09;
  } else {
    switch (currentstep / (MICROSTEPS / 2)) {
    case 0:
      latch_state |= 0x1; // energize coil 1 only
      break;
    case 1:
      latch_state |= 0x3; // energize coil 1+2
      break;
    case 2:
      latch_state |= 0x2; // energize coil 2 only
      break;
    case 3:
      latch_state |= 0x6; // energize coil 2+3
      break;
    case 4:
      latch_state |= 0x4; // energize coil 3 only
      break;
    case 5:
      latch_state |= 0xC; // energize coil 3+4
      break;
    case 6:
      latch_state |= 0x8; // energize coil 4 only
      break;
    case 7:
      latch_state |= 0x9; // energize coil 1+4
      break;
    }
  }
#ifdef MOTORDEBUG
  Serial.print("Latch: 0x");
  Serial.println(latch_state, HEX);
#endif

  if (latch_state & 0x1) {
    // Serial.println(AIN2pin);
    MC->setPin(AIN2pin, HIGH);
  } else {
    MC->setPin(AIN2pin, LOW);
  }
  if (latch_state & 0x2) {
    MC->setPin(BIN1pin, HIGH);
    // Serial.println(BIN1pin);
  } else {
    MC->setPin(BIN1pin, LOW);
  }
  if (latch_state & 0x4) {
    MC->setPin(AIN1pin, HIGH);
    // Serial.println(AIN1pin);
  } else {
    MC->setPin(AIN1pin, LOW);
  }
  if (latch_state & 0x8) {
    MC->setPin(BIN2pin, HIGH);
    // Serial.println(BIN2pin);
  } else {
    MC->setPin(BIN2pin, LOW);
  }

  return currentstep;
}
//  -- END OF FILE --


#pragma once

/*!
 * @file CA_MotorShield.h
 *
 * @mainpage CA MotorShield Library
 *
 * @section intro_sec Introduction
 *
 * This library is designed for the CA MotorShield, an enhanced and integrated 
 * solution combining the features of Adafruit Motor Shield V2 and the PCA9685_RT 
 * library. It addresses specific challenges such as I2C communication errors and 
 * motor rotation direction issues observed with the Adafruit MotorShield v2. 
 * This library supports DC motors, Stepper motors with microstepping, and is capable 
 * of stacking multiple shields or wings for expanded functionality.
 * 
 * It utilizes I2C for communication, requiring 2 pins (SCL+SDA) for interfacing.
 * 
 * This project represents a commitment to open-source hardware and software by 
 * integrating and enhancing existing resources. Your support for this project, 
 * through the use of CA MotorShield and contributions to its development, is 
 * greatly appreciated.
 *
 * @section author Author
 *
 * Developed by CodingArray / hanol JU, based on the original work by Limor Fried/Ladyada 
 * for Adafruit Industries and enhancements from Rob Tillaart's PCA9685_RT library.
 *
 * @section license License
 *
 * This library is released under the MIT license, to reflect the integration of the PCA9685_RT 
 * library's features and the enhancements made to the original Adafruit MotorShield v2 library.
 * The Adafruit MotorShield V2 is distributed under the BSD license. It is essential that all 
 * redistribution includes this and the original text of the licenses.
 * 
 * URL: https://github.com/CodingArray/CA_MotorShield_V1_Library
 */

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "Wire.h"

#define MOTORDEBUG 0

#define MICROSTEPS 16  // 8 or 16

#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4

#define SINGLE 1
#define DOUBLE 2
#define INTERLEAVE 3
#define MICROSTEP 4

// ERROR CODES
#define PCA9685_OK                  0x00
#define PCA9685_ERROR               0xFF
#define PCA9685_ERR_CHANNEL         0xFE
#define PCA9685_ERR_MODE            0xFD
#define PCA9685_ERR_I2C             0xFC

//  get/setFrequency()
#define PCA9685_MIN_FREQ              24
#define PCA9685_MAX_FREQ            1526

//  REGISTERS CONFIGURATION - check datasheet for details
#define PCA9685_MODE1               0x00
#define PCA9685_MODE2               0x01

//  Configuration bits MODE1 register
#define PCA9685_MODE1_RESTART       0x80  //  0 = disable      1 = enable
#define PCA9685_MODE1_EXTCLK        0x40  //  0 = internal     1 = external (datasheet)
#define PCA9685_MODE1_AUTOINCR      0x20  //  0 = disable      1 = enable
#define PCA9685_MODE1_SLEEP         0x10  //  0 = normal       1 = sleep
#define PCA9685_MODE1_SUB1          0x08  //  0 = disable      1 = enable
#define PCA9685_MODE1_SUB2          0x04  //  0 = disable      1 = enable
#define PCA9685_MODE1_SUB3          0x02  //  0 = disable      1 = enable
#define PCA9685_MODE1_ALLCALL       0x01  //  0 = disable      1 = enable
#define PCA9685_MODE1_NONE          0x00

//  Configuration bits MODE2 register
#define PCA9685_MODE2_INVERT        0x10  //  0 = normal       1 = inverted
#define PCA9685_MODE2_ACK           0x08  //  0 = on STOP      1 = on ACK
#define PCA9685_MODE2_TOTEMPOLE     0x04  //  0 = open drain   1 = totem-pole
#define PCA9685_MODE2_OUTNE         0x03  //  datasheet
#define PCA9685_MODE2_NONE          0x00

//  (since 0.4.0)
#define PCA9685_SUBADR(x)           (0x01 + (x))  //  x = 1..3
#define PCA9685_ALLCALLADR          0x05

// REGISTERS - CHANNELS
//  0x06 + 4*channel is base per channel
#define PCA9685_CHANNEL_0           0x06
#define PCA9685_CHANNEL(x)          (0x06 + ((x) * 4))  //  x = 0..15

//  REGISTERS - ALL_ON ALL_OFF - partly implemented
#define PCA9685_ALL_ON_L            0xFA
#define PCA9685_ALL_ON_H            0xFB
#define PCA9685_ALL_OFF_L           0xFC
#define PCA9685_ALL_OFF_H           0xFD   //  used for allOFF()

// REGISTERS - FREQUENCY
#define PCA9685_PRE_SCALER          0xFE

//  NOT IMPLEMENTED
//  WARNING: DO NOT USE THIS REGISTER (see datasheet)
#define PCA9685_TESTMODE            0xFF   //  do not be use. see datasheet.


class CA_MotorShield;

// CA_DCMotor class
class CA_DCMotor {
public:
  CA_DCMotor(void);

  friend class CA_MotorShield; ///< Let MotorShield create DCMotors

  void run(uint8_t);
  void setSpeed(uint16_t);
  void setSpeedNoLimit(uint16_t);
  void setSpeedFine(uint16_t speed);

private:
  uint8_t PWMpin, IN1pin, IN2pin;
  CA_MotorShield *MC;
  uint8_t motornum;
};

// CA_StepperMotor class
class CA_StepperMotor {
public:
  CA_StepperMotor(void);

  friend class CA_MotorShield; ///< Let MotorShield create StepperMotors

  void setSpeed(uint16_t);
  void step(uint16_t steps, uint8_t dir, uint8_t style = SINGLE);
  uint8_t onestep(uint8_t dir, uint8_t style);
  void release(void);

private:
  uint32_t usperstep;

  uint8_t PWMApin, AIN1pin, AIN2pin;
  uint8_t PWMBpin, BIN1pin, BIN2pin;
  uint16_t revsteps; // # steps per revolution
  uint8_t currentstep;
  CA_MotorShield *MC;
  uint8_t steppernum;
};

// CA_MotorShield class
class CA_MotorShield {
public:

  explicit CA_MotorShield(const uint8_t deviceAddress = 0x60, TwoWire *wire = &Wire);
  bool     begin();
  bool     begin(uint16_t pwmFrequency); 

  bool     isConnected();
  uint8_t  getAddress();

  /////////////////////////////////////////////////////
  //
  //  CONFIGURATION
  //
  uint8_t  configure(uint8_t mode1_mask, uint8_t mode2_mask);
  uint8_t  channelCount();

  //  reg = 1, 2  check datasheet for values
  uint8_t  writeMode(uint8_t reg, uint8_t value);
  uint8_t  readMode(uint8_t reg);

  //  convenience wrappers
  uint8_t  setMode1(uint8_t value);
  uint8_t  setMode2(uint8_t value);
  uint8_t  getMode1();
  uint8_t  getMode2();

  /////////////////////////////////////////////////////
  //
  //  WRITE / SETPWM
  //
  //  single PWM setting, channel = 0..15,
  //  onTime = 0..4095, offTime = 0..4095
  //  allows shifted PWM's e.g. 2 servo's that do not start at same time.
  //         this will distribute the (peak) load
  void     setPWM(uint8_t channel, uint16_t onTime, uint16_t offTime);
  void     getPWM(uint8_t channel, uint16_t* onTime, uint16_t* offTime);

  // single PWM setting, channel = 0..15, offTime = 0..4095  (onTime = 0)
  void     setPWM(uint8_t channel, uint16_t offTime);

  void     setPin(uint8_t pin, boolean val);

  //  set update frequency for all channels
  //  freq = 24 - 1526 Hz
  //  note: as the frequency is converted to an 8 bit pre-scaler
  //       the frequency set will seldom be exact, but best effort.
  void     setFrequency(uint16_t freq, int offset = 0);
  int      getFrequency(bool cache = true);

  //  set channel  HIGH or LOW (effectively no PWM)
  void     write1(uint8_t channel, uint8_t mode);

  //  for backwards compatibility; will be removed in future
  void     setON(uint8_t channel)   { write1(channel, HIGH); };
  void     setOFF(uint8_t channel)  { write1(channel, LOW); };

//  experimental for 0.3.0
  void     allChannelInitialize();
  //  experimental for 0.3.0
  void     allOFF();

  /////////////////////////////////////////////////////
  //
  //  SUB CALL  -  ALL CALL  (since 0.4.0)
  //
  //  nr = { 1, 2, 3 }
  bool     enableSubCall(uint8_t nr);
  bool     disableSubCall(uint8_t nr);
  bool     isEnabledSubCall(uint8_t nr);
  bool     setSubCallAddress(uint8_t nr, uint8_t address);
  uint8_t  getSubCallAddress(uint8_t nr);

  bool     enableAllCall();
  bool     disableAllCall();
  bool     isEnabledAllCall();
  bool     setAllCallAddress(uint8_t address);
  uint8_t  getAllCallAddress();

  /////////////////////////////////////////////////////
  //
  //  OE - Output Enable control
  //
  bool     setOutputEnablePin(uint8_t pin);
  bool     setOutputEnable(bool on);
  uint8_t  getOutputEnable();

  /////////////////////////////////////////////////////
  //
  //  ERROR
  //
  //  note error flag is reset after read!
  int      lastError();


  //  EXPERIMENTAL 0.4.2
  int I2C_SoftwareReset(uint8_t method);  //  0 or 1

///////////////////////////////////////////////////////////////////////
  friend class CA_DCMotor; ///< Let DCMotors control the Shield

  CA_DCMotor *getMotor(uint8_t n);
  CA_StepperMotor *getStepper(uint16_t steps, uint8_t n);
///////////////////////////////////////////////////////////////////////

private:
  //  DIRECT CONTROL
  void    writeReg(uint8_t reg, uint8_t value);
  void    writeReg2(uint8_t reg, uint16_t a, uint16_t b);
  uint8_t readReg(uint8_t reg);

  uint8_t _address = 0x60;
  int     _error;
  int     _freq = 1600;  //  default PWM frequency - P25 datasheet
  uint8_t _channelCount = 16;
  uint8_t _OutputEnablePin;

///////////////////////////////////////////////////////////////////////
  CA_DCMotor dcmotors[4];
  CA_StepperMotor steppers[2];
///////////////////////////////////////////////////////////////////////

  TwoWire*  _wire;
};
//  -- END OF FILE --
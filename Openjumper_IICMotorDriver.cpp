/*
 * @Author: Mr.Jiang
 * @Date: 2021-03-23 16:29:51
 * @LastEditors: Mr.Jiang
 * @LastEditTime: 2021-03-26 11:06:11
 * @Description: openjumper_IICmotordrive
 */

#include "Openjumper_IICMotorDriver.h"
#include <Wire.h>
#if defined(__AVR__)
#define WIRE Wire
#elif defined(CORE_TEENSY) // Teensy boards
#define WIRE Wire
#else // Arduino Due
#define WIRE Wire
#endif

Openjumper_IICMotorDriver::Openjumper_IICMotorDriver(uint8_t addr)
{
  _i2caddr = addr;
}

void Openjumper_IICMotorDriver::begin(void)
{
  WIRE.begin();
  reset();
  setPWMFreq(60);
}

void Openjumper_IICMotorDriver::reset(void)
{
  write8(PCA9685_MODE1, 0x0);
}

/**
 * @description: 
 * @param {float} freq 设置运行频率
 * @return {*}
 */
void Openjumper_IICMotorDriver::setPWMFreq(float freq)
{
  //Serial.print("Attempting to set freq ");
  //Serial.println(freq);
  freq *= 0.9; // Correct for overshoot in the frequency setting (see issue #11).
  float prescaleval = 25000000;
  prescaleval /= 4096;
  prescaleval /= freq;
  prescaleval -= 1;
  uint8_t prescale = floor(prescaleval + 0.5);
  uint8_t oldmode = read8(PCA9685_MODE1);
  uint8_t newmode = (oldmode & 0x7F) | 0x10; // sleep
  write8(PCA9685_MODE1, newmode);            // go to sleep
  write8(PCA9685_PRESCALE, prescale);        // set the prescaler
  write8(PCA9685_MODE1, oldmode);
  delay(5);
  write8(PCA9685_MODE1, oldmode | 0xa1); //  This sets the MODE1 register to turn on auto increment.
                                         // This is why the beginTransmission below was not working.
  //  Serial.print("Mode now 0x"); Serial.println(read8(PCA9685_MODE1), HEX);
}

void Openjumper_IICMotorDriver::setPWM(uint8_t num, uint16_t on, uint16_t off)
{
  //Serial.print("Setting PWM "); Serial.print(num); Serial.print(": "); Serial.print(on); Serial.print("->"); Serial.println(off);

  WIRE.beginTransmission(_i2caddr);
  WIRE.write(LED0_ON_L + 4 * num);
  WIRE.write(on);
  WIRE.write(on >> 8);
  WIRE.write(off);
  WIRE.write(off >> 8);
  WIRE.endTransmission();
}

// Sets pin without having to deal with on/off tick placement and properly handles
// a zero value as completely off.  Optional invert parameter supports inverting
// the pulse for sinking to ground.  Val should be a value from 0 to 4095 inclusive.
/**
 * @description: 
 * @param {uint8_t} num
 * @param {uint16_t} val
 * @param {bool} invert
 * @return {*}
 */
void Openjumper_IICMotorDriver::setPin(uint8_t num, uint16_t val, bool invert)
{
  // Clamp value between 0 and 4095 inclusive.
  val =((val)<(4095)?(val):(4095));
  if (invert)
  {
    if (val == 0)
    {
      // Special value for signal fully on.
      setPWM(num, 4096, 0);
    }
    else if (val == 4095)
    {
      // Special value for signal fully off.
      setPWM(num, 0, 4096);
    }
    else
    {
      setPWM(num, 0, 4095 - val);
    }
  }
  else
  {
    if (val == 4095)
    {
      // Special value for signal fully on.
      setPWM(num, 4096, 0);
    }
    else if (val == 0)
    {
      // Special value for signal fully off.
      setPWM(num, 0, 4096);
    }
    else
    {
      setPWM(num, 0, val);
    }
  }
}
/**
 * @description: 
 * @param {*}
 * @return {*}
 */
void Openjumper_IICMotorDriver::motorConfig(int8_t offsetM1, int8_t offsetM2, int8_t offsetM3,
                                            int8_t offsetM4)
{
  offsetM1 = offsetM1 >= 0 ? DIRP : DIRN; // 限制正反方向值 1、-1
  offsetM2 = offsetM2 >= 0 ? DIRP : DIRN; // 限制正反方向值 1、-1
  offsetM3 = offsetM3 >= 0 ? DIRP : DIRN; // 限制正反方向值 1、-1
  offsetM4 = offsetM4 >= 0 ? DIRP : DIRN; // 限制正反方向值 1、-1

  _OFFSETM1 = offsetM1;
  _OFFSETM2 = offsetM2;
  _OFFSETM3 = offsetM3;
  _OFFSETM4 = offsetM4;
}
/**
 * @description: 
 * @param {uint8_t} _in1Pin
 * @param {uint8_t} _in2Pin
 * @param {uint8_t} _pwmPin
 * @param {int16_t} _mspeed
 * @param {int8_t} _moffset
 * @return {*}
 */
void Openjumper_IICMotorDriver::driverOneMotor_IIC(uint8_t _in1Pin, uint8_t _in2Pin, uint8_t _pwmPin, int16_t _mspeed, int8_t _moffset = 1)
{
  _moffset = _moffset >= 0 ? DIRP: DIRN; // 限制正反方向值 1、-1
  _mspeed = _mspeed * _moffset;
  if (_mspeed > 0)
  {
    setPin(_in1Pin, 4095, 0);
    setPin(_in2Pin, 0, 0);
    setPin(_pwmPin, _mspeed, 0);
  }
  else if (_mspeed < 0)
  {
    setPin(_in1Pin, 0, 0);
    setPin(_in2Pin, 4095, 0);
    setPin(_pwmPin, 0 - _mspeed, 0);
  }
  else
  {
    setPin(_in1Pin, 0, 0);
    setPin(_in2Pin, 0, 0);
  }
}

/**
 * @description: 
 * @param {uint8_t} num
 * @param {uint8_t} _mspeed
 * @return {*}
 */
void Openjumper_IICMotorDriver::setMotor(uint8_t num, int16_t _mspeed)
{

  if (num == M1)
  { // MOTOR 1
    driverOneMotor_IIC(_M1IN1, _M1IN2, _M1PWM, _mspeed, _OFFSETM1);
  }
  else if (num == M2)
  { // MOTOR 2
    driverOneMotor_IIC(_M2IN1, _M2IN2, _M2PWM, _mspeed, _OFFSETM2);
  }
  else if (num == M3)
  { // MOTOR 3
    driverOneMotor_IIC(_M3IN1, _M3IN2, _M3PWM, _mspeed, _OFFSETM3);
  }
  else if (num == M4)
  { // MOTOR 4
    driverOneMotor_IIC(_M4IN1, _M4IN2, _M4PWM, _mspeed, _OFFSETM4);
  }
  else if (num == MAll)
  {
    driverOneMotor_IIC(_M1IN1, _M1IN2, _M1PWM, _mspeed, _OFFSETM1);
    driverOneMotor_IIC(_M2IN1, _M2IN2, _M2PWM, _mspeed, _OFFSETM2);
    driverOneMotor_IIC(_M3IN1, _M3IN2, _M3PWM, _mspeed, _OFFSETM3);
    driverOneMotor_IIC(_M4IN1, _M4IN2, _M4PWM, _mspeed, _OFFSETM4);
  }
}

/**
 * @description: 
 * @param {int16_t} speedall
 * @return {*}
 */
void Openjumper_IICMotorDriver::setAllMotor(int16_t _mspeed)
{
  driverOneMotor_IIC(_M1IN1, _M1IN2, _M1PWM, _mspeed, _OFFSETM1);
  driverOneMotor_IIC(_M2IN1, _M2IN2, _M2PWM, _mspeed, _OFFSETM2);
  driverOneMotor_IIC(_M3IN1, _M3IN2, _M3PWM, _mspeed, _OFFSETM3);
  driverOneMotor_IIC(_M4IN1, _M4IN2, _M4PWM, _mspeed, _OFFSETM4);
}
void Openjumper_IICMotorDriver::setAllMotor(int16_t _mspeedM1,int16_t _mspeedM2,int16_t _mspeedM3,int16_t _mspeedM4)
{
  driverOneMotor_IIC(_M1IN1, _M1IN2, _M1PWM, _mspeedM1, _OFFSETM1);
  driverOneMotor_IIC(_M2IN1, _M2IN2, _M2PWM, _mspeedM2, _OFFSETM2);
  driverOneMotor_IIC(_M3IN1, _M3IN2, _M3PWM, _mspeedM3, _OFFSETM3);
  driverOneMotor_IIC(_M4IN1, _M4IN2, _M4PWM, _mspeedM4, _OFFSETM4);
}
void Openjumper_IICMotorDriver::stopMotor(uint8_t _mNum)
{
  if (_mNum == M1)
  { // MOTOR 1
    setPin(_M1PWM, 0, 0);
  }
  else if (_mNum == M2)
  { // MOTOR 2
    setPin(_M2PWM, 0, 0);
  }
  else if (_mNum == M3)
  { // MOTOR 3
    setPin(_M3PWM, 0, 0);
  }
  else if (_mNum == M4)
  { // MOTOR 4
    setPin(_M4PWM, 0, 0);
  }
  else if (_mNum == MAll)
  { // MOTOR 1 2 3 4
    setPin(_M1PWM, 0, 0);
    setPin(_M2PWM, 0, 0);
    setPin(_M3PWM, 0, 0);
    setPin(_M4PWM, 0, 0);
  }
}

/**
 * @description:  
 * @param {uint8_t} n
 * @param {double} pulse
 * @return {*}
 */
void Openjumper_IICMotorDriver::setServoPulse(uint8_t n, double pulse)
{
  n = n > 4 ? 4 : n < 1 ? 1 : n;

  setPWM(s_pin[n - 1], 0, pulse);
}

void Openjumper_IICMotorDriver::digitalWrite(uint8_t n, uint8_t pulse)
{
  
  n = n > 4 ? 4 : n < 1 ? 1 : n;
if(pulse){
  setPWM(s_pin[n - 1],4096, 0);
}else{
  setPWM(s_pin[n - 1], 0,4096);
}
  
}
/**
 * @description: 
 * @param {uint8_t} addr IIc 驱动地址
 * @return {*}
 */

uint8_t Openjumper_IICMotorDriver::read8(uint8_t addr)
{
  WIRE.beginTransmission(_i2caddr);
  WIRE.write(addr);
  WIRE.endTransmission();

  WIRE.requestFrom((uint8_t)_i2caddr, (uint8_t)1);
  return WIRE.read();
}

void Openjumper_IICMotorDriver::write8(uint8_t addr, uint8_t d)
{
  WIRE.beginTransmission(_i2caddr);
  WIRE.write(addr);
  WIRE.write(d);
  WIRE.endTransmission();
}

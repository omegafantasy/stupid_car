/*
 * @Author: Mr.Jiang
 * @Date: 2021-03-23 16:29:51
 * @LastEditors: Mr.Jiang
 * @LastEditTime: 2021-03-25 18:21:33
 * @Description: openjumper_IICmotordrive
 */

#ifndef _Openjumper_IICMotorDriver_H
#define _Openjumper_IICMotorDriver_H

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4

#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE

#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9

#define ALLLED_ON_L 0xFA
#define ALLLED_ON_H 0xFB
#define ALLLED_OFF_L 0xFC
#define ALLLED_OFF_H 0xFD

#define M1 1
#define M2 2
#define M3 3
#define M4 4
#define MAll 5 //所有电机

#define _M1IN1 0  // 电机M1 输入1
#define _M1IN2 1  // 电机M1 输入2
#define _M1PWM 2  // 电机M1 PWM
#define _M2IN1 3  // 电机M2 输入1
#define _M2IN2 4  // 电机M2 输入2
#define _M2PWM 5  // 电机M2 PWM
#define _M3IN1 8  // 电机M3 输入1
#define _M3IN2 9  // 电机M3 输入2
#define _M3PWM 10 // 电机M3 PWM
#define _M4IN1 11 // 电机M4 输入1
#define _M4IN2 12 // 电机M4 输入2
#define _M4PWM 13 // 电机M4 PWM

#define DIRP 1
#define DIRN -1

#define S1 1
#define S2 2
#define S3 3
#define S4 4

/**
 * @description: 
 * @param {*}
 * @return {*}
 */
class Openjumper_IICMotorDriver
{
public:
  Openjumper_IICMotorDriver(uint8_t addr = 0x40);
  void begin(void);
  void reset(void);
  void setPWMFreq(float freq);
  void setPWM(uint8_t num, uint16_t on, uint16_t off);
  void setPin(uint8_t num, uint16_t val, bool invert = false);

  void motorConfig(int8_t offsetM1, int8_t offsetM2, int8_t offsetM3, int8_t offsetM4); //配置运行方向

  void setMotor(uint8_t num, int16_t _mspeed);
  void setAllMotor(int16_t _mspeed);
  void setAllMotor(int16_t _mspeedM1,int16_t _mspeedM2,int16_t _mspeedM3,int16_t _mspeedM4);
  void stopMotor(uint8_t _mNum); // 刹车
  void setServoPulse(uint8_t n, double pulse); //设置舵机角度
  void digitalWrite(uint8_t n, uint8_t pulse);  //设置引脚输出

private:
  uint8_t _i2caddr;
  uint8_t s_pin[4] = {6, 7, 14, 15};
  uint8_t read8(uint8_t addr);
  void write8(uint8_t addr, uint8_t d);
  void driverOneMotor_IIC(uint8_t _in1Pin, uint8_t _in2Pin, uint8_t _pwmPin, int16_t _mspeed, int8_t _moffset);

  int8_t _OFFSETM1 = 1; /** motor M1 reverse 电机M1反向 **/
  int8_t _OFFSETM2 = 1;     /** motor M2 reverse 电机M2反向 **/
  int8_t _OFFSETM3 = 1;     /** motor M3 reverse 电机M3反向 **/
  int8_t _OFFSETM4 = 1;     /** motor M4 reverse 电机M4反向 **/
};

#endif

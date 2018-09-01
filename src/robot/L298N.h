/*
 * L298N Library
 *
 * Felipe Kühne
 * 03/05/2017
 * 
 * 28/08/2018: Updated to run on ESP platform
 * 
 */

#ifndef __L298N_H__
#define __L298N_H__

#include <Arduino.h>

/* Let us configure the PWM variables here. */
// use first two channels of 16 channels (started from zero)
#define LEDC_CHANNEL_0 0
#define LEDC_CHANNEL_1 1
// use 13 bit precision for LEDC timer
#define LEDC_TIMER_13_BIT 13
// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ 5000

enum motorDirection {FW = 0, BW};
enum motorState {STOP = 0, RUN};

class DCMotor
{
public:
  DCMotor();
  /* This is when the H bridge has only one signal to control the direction,
   *   that is, the two signals needed to operate the H bridge are generated
   *   by hardware. */
  DCMotor(const int enable, const int direction);

  int setBreakPin(const int pin);
  
  int enable_pin;
  int dir_pin;
  int break_pin;

  motorDirection _motorDirection;
  motorState _motorState;
};

class L298N
{
public:
  L298N(DCMotor& motor1, DCMotor& motor2);
  int setState(const motorState state);
  int setDutyCycle(unsigned int dutyCycle1, unsigned int dutyCycle2);
  int setDutyCycle(const unsigned int dutyCycle);
  int setDirection(DCMotor& motor, const motorDirection direction);
  int setDirection(const motorDirection direction);

private:
  DCMotor _motor1, _motor2;
  motorState _motorState;
  int pwmChannel1, pwmChannel2;
};

#endif /* __L298N_H__ */

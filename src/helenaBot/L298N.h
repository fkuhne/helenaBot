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

#if(ESP_PLATFORM)
  /* Let us configure the PWM variables here. */
  // use first two channels of 16 channels (started from zero)
  #define LEDC_CHANNEL_0 0
  #define LEDC_CHANNEL_1 1
  // use 13 bit precision for LEDC timer
  #define LEDC_TIMER_13_BIT 13
  // use 5000 Hz as a LEDC base frequency
  #define LEDC_BASE_FREQ 5000
#endif

enum motorDirection {FW = 0, BW};
enum motorState {STOP = 0, RUN};

class DCMotor
{
public:
  DCMotor();
  /* This is when the H bridge has two signals to control the direction. */
  DCMotor(const int enable, const int inA, const int inB);
  /* This is when the H bridge has only one signal to control the direction,
   *   that is, the two signals needed to operate the H bridge are generated
   *   by hardware. */
  DCMotor(const int enable, const int direction);
  
  int enable_pin;
  int inA_pin;
  int inB_pin;

  motorDirection _motorDirection;
  motorState _motorState;
};

class L298N
{
public:
  L298N(const DCMotor& motor);
  L298N(const DCMotor& motor1, const DCMotor& motor2);

  int setState(DCMotor& motor, const motorState state);
  int setState(const motorState state);
#if(!ESP_PLATFORM)
  /* Due to PWM channel definitions, I can only set the duty cycle without
   *   knowing the motor when the platform is not ESP. */
  int setDutyCycle(DCMotor& motor, const unsigned int dutyCycle);
#endif
  int setDutyCycle(const unsigned int dutyCycle1, const unsigned int dutyCycle2);
  int setDutyCycle(const unsigned int dutyCycle);
  int setDirection(DCMotor& motor, const motorDirection direction);
  int setDirection(const motorDirection direction);

private:
  bool isMotor2Defined;
  DCMotor _motor1, _motor2;
#if(ESP_PLATFORM)
  int pwmChannel1, pwmChannel2;
#endif
};

#endif /* __L298N_H__ */

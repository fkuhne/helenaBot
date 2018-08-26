/*
 * L298N Library
 *
 * Felipe Kühne
 * 03/05/2017
 */

#ifndef __L298N_H__
#define __L298N_H__

#include <Arduino.h>

enum motorDirection {FW = 0, BW};
enum motorState {STOP = 0, RUN};

class DCMotor
{
public:
  DCMotor() {};
  DCMotor(const int enable, const int inA, const int inB);

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
  int setDutyCycle(DCMotor& motor, const unsigned int dutyCycle);
  int setDutyCycle(const unsigned int dutyCycleLeft, const unsigned int dutyCycleRight);
  int setDutyCycle(const unsigned int dutyCycle);
  int setDirection(DCMotor& motor, const motorDirection direction);
  int setDirection(const motorDirection direction);

private:
  bool isMotor2Defined;
  DCMotor _motor1, _motor2;
};

#endif /* __L298N_H__ */

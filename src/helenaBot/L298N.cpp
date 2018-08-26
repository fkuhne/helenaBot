/*
 * L298N Library
 *
 * Felipe Kühne
 * 03/05/2017
 */

#include "L298N.h"

DCMotor::DCMotor(const int enable, const int inA, const int inB)
{
  enable_pin = enable;
  inA_pin = inA;
  inB_pin = inB;

  _motorState = STOP;
}

L298N::L298N(const DCMotor& motor)
{
  pinMode(motor.enable_pin, OUTPUT);
  digitalWrite(motor.enable_pin, LOW);

  pinMode(motor.inA_pin, OUTPUT);
  digitalWrite(motor.inA_pin, LOW);

  pinMode(motor.inB_pin, OUTPUT);
  digitalWrite(motor.inB_pin, LOW);

  _motor1 = motor;
  isMotor2Defined = false;
};

L298N::L298N(const DCMotor& motor1, const DCMotor& motor2)
{
  pinMode(motor1.enable_pin, OUTPUT);
  digitalWrite(motor1.enable_pin, LOW);

  pinMode(motor1.inA_pin, OUTPUT);
  digitalWrite(motor1.inA_pin, LOW);

  pinMode(motor1.inB_pin, OUTPUT);
  digitalWrite(motor1.inB_pin, LOW);

  /* Copy to internal object. */
  _motor1 = motor1;

  pinMode(motor2.enable_pin, OUTPUT);
  digitalWrite(motor2.enable_pin, LOW);

  pinMode(motor2.inA_pin, OUTPUT);
  digitalWrite(motor2.inA_pin, LOW);

  pinMode(motor2.inB_pin, OUTPUT);
  digitalWrite(motor2.inB_pin, LOW);

  /* Copy to internal object. */
  _motor2 = motor2;
  isMotor2Defined = true;
}

int L298N::setState(DCMotor& motor, const motorState state)
{
  if(motor.enable_pin == 0)
	  return -1;

  if(state == STOP)
  {
    digitalWrite(motor.enable_pin, LOW);
    motor._motorState = state;
  }

  else if(state == RUN)
  {
    digitalWrite(motor.enable_pin, HIGH);
    motor._motorState = state;
  }

  else
    return -1;

  return 0;
}

int L298N::setState(const motorState state)
{
  int ret = setState(_motor1, state);
  if(!ret && isMotor2Defined)
    ret = setState(_motor2, state);

  return ret;
}

int L298N::setDutyCycle(DCMotor& motor, const unsigned int dutyCycle)
{
  /* Test for invalid parameters. */
  if(motor.enable_pin == 0 || dutyCycle > 255)
    return -1;

  if(dutyCycle == 0)
    return setState(motor, STOP);

  analogWrite(motor.enable_pin, dutyCycle);

  return 0;
}

int L298N::setDutyCycle(const unsigned int dutyCycleLeft, const unsigned int dutyCycleRight)
{
  int ret = setDutyCycle(_motor1, dutyCycleLeft);
  if(!ret && isMotor2Defined)
    ret = setDutyCycle(_motor2, dutyCycleRight);

  return ret;
}

int L298N::setDutyCycle(const unsigned int dutyCycle)
{
  int ret = setDutyCycle(_motor1, dutyCycle);
  if(!ret && isMotor2Defined)
    ret = setDutyCycle(_motor2, dutyCycle);

  return ret;
}

int L298N::setDirection(DCMotor& motor, const motorDirection direction)
{
  if(motor.inA_pin == 0 || motor.inB_pin == 0)
	  return -1;

  if(direction == FW)
  {
    digitalWrite(motor.inA_pin, LOW);
    digitalWrite(motor.inB_pin, HIGH);
    motor._motorDirection = direction;
  }
  else if(direction == BW)
  {
    digitalWrite(motor.inA_pin, HIGH);
    digitalWrite(motor.inB_pin, LOW);
    motor._motorDirection = direction;
  }
  else
    return -1;

  return 0;
}

int L298N::setDirection(const motorDirection direction)
{
  int ret = setDirection(_motor1, direction);
  if(!ret && isMotor2Defined)
    ret = setDirection(_motor2, direction);

  return ret;
}

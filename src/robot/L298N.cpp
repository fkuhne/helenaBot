/*
 * L298N Library
 *
 * Felipe Kühne
 * 03/05/2017
 * 
 * 28/08/2018: Updated to run on ESP platform
 * 
 */

#include "L298N.h"

DCMotor::DCMotor()
{
  enable_pin = -1;
  dir_pin = -1;
  _motorState = STOP;
}

/* This is when we have an H-bridge with only one exposed pin
 *   to set the direction (that is, the inversion of the second
 *   channel (for the same bridge size) is implemented via
 *   hardware). */
DCMotor::DCMotor(const int enable, const int direction)
{
  enable_pin = enable;
  dir_pin = direction;
  _motorState = STOP;
}

/*DCMotor::setBreakPin(const int pin)
{
  break_pin = pin;
}*/

L298N::L298N(DCMotor& motor1, DCMotor& motor2)
{
  pinMode(motor1.enable_pin, OUTPUT);
  digitalWrite(motor1.enable_pin, LOW);
  pinMode(motor2.enable_pin, OUTPUT);
  digitalWrite(motor2.enable_pin, LOW);
  
  pinMode(motor1.dir_pin, OUTPUT);
  digitalWrite(motor1.dir_pin, LOW);
  pinMode(motor2.dir_pin, OUTPUT);
  digitalWrite(motor2.dir_pin, LOW);

  /* Copy to internal objects. */
  _motor1 = motor1;
  _motor2 = motor2;
}

int L298N::setState(const motorState state)
{
  if(_motor1.enable_pin < 0 || _motor2.enable_pin < 0)
    return -1;

  //if(_motorState == state)
    //return 0;
    
  if(state == STOP) 
  {
    digitalWrite(_motor1.enable_pin, LOW);
    digitalWrite(_motor2.enable_pin, LOW);
  }
  else if(state == RUN) 
  {
    digitalWrite(_motor1.enable_pin, HIGH);
    digitalWrite(_motor2.enable_pin, HIGH);
  }
  
  _motor1._motorState = state;
  _motor2._motorState = state;
  _motorState = state;

  return 0;
}

int L298N::setDutyCycle(unsigned int dutyCycle1, unsigned int dutyCycle2)
{
  if(_motor1.enable_pin < 0 || _motor2.enable_pin < 0)
    return -1;

  dutyCycle1 = constrain(dutyCycle1, 0, 255);
  dutyCycle2 = constrain(dutyCycle2, 0, 255);

  if(dutyCycle1 == 0) digitalWrite(_motor1.enable_pin, LOW);
  else analogWrite(_motor1.enable_pin, dutyCycle1);
    
  if(dutyCycle2 == 0) digitalWrite(_motor2.enable_pin, LOW);
  else analogWrite(_motor2.enable_pin, dutyCycle2);

  return 0;
}

int L298N::setDutyCycle(const unsigned int dutyCycle)
{
  return setDutyCycle(dutyCycle, dutyCycle);
}

int L298N::setDirection(DCMotor& motor, const motorDirection direction)
{
  if(motor.dir_pin < 0)
	  return -1;

  if(direction == FW) digitalWrite(motor.dir_pin, HIGH);

  else if(direction == BW) digitalWrite(motor.dir_pin, LOW);

  else return -1;

  motor._motorDirection = direction;

  return 0;
}

int L298N::setDirection(const motorDirection direction)
{
  int ret = setDirection(_motor1, direction);
  ret = setDirection(_motor2, direction);
  
  return ret;
}


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

/* FK: Stolen from File/Examples/ESP32/AnalogOut/LEDCSoftwareFade */
// Arduino like analogWrite
// value has to be between 0 and valueMax
void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 350)
{
  // calculate duty, 8191 from 2 ^ 13 - 1
  uint32_t duty = (8191 / valueMax) * min(value, valueMax);
  // write duty to LEDC
  ledcWrite(channel, duty);
}

DCMotor::DCMotor()
{
  enable_pin = -1;
  dir_pin = -1;
  break_pin = -1;
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
  // Setup timer and attach timer to PWM pin
  pwmChannel1 = LEDC_CHANNEL_0;
  ledcSetup(pwmChannel1, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
  ledcAttachPin(motor1.enable_pin, pwmChannel1);
  ledcAnalogWrite(pwmChannel1, 0);

  /* Copy to internal object. */
  _motor1 = motor1;

  pwmChannel2 = LEDC_CHANNEL_1;
  // Setup timer and attach timer to PWM pin
  ledcSetup(pwmChannel2, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
  ledcAttachPin(motor2.enable_pin, pwmChannel2);
  ledcAnalogWrite(pwmChannel2, 0);

  /* Copy to internal object. */
  _motor2 = motor2;

  /* Configure direction pins. */
  pinMode(motor1.dir_pin, OUTPUT);
  digitalWrite(motor1.dir_pin, LOW);
  pinMode(motor2.dir_pin, OUTPUT);
  digitalWrite(motor2.dir_pin, LOW);
}

int L298N::setState(const motorState state)
{
  if(_motor1.enable_pin < 0 || _motor2.enable_pin < 0)
    return -1;

  if(state == STOP) 
  {
    ledcAnalogWrite(pwmChannel1, 0);
    ledcAnalogWrite(pwmChannel2, 0);
  }
  else if(state == RUN) 
  {
    ledcAnalogWrite(pwmChannel1, 255);
    ledcAnalogWrite(pwmChannel2, 255);
  }
  
  _motor1._motorState = _motor2._motorState = state;
  return 0;
}

int L298N::setDutyCycle(const unsigned int dutyCycle1, const unsigned int dutyCycle2)
{
  /* MOTOR 1 */
  if(_motor1.enable_pin < 0 || dutyCycle1 > 255)
    return -1;

  if(dutyCycle1 == 0) ledcAnalogWrite(pwmChannel1, 0);
  else ledcAnalogWrite(pwmChannel1, dutyCycle1);

  /* MOTOR 2 */
  if(_motor2.enable_pin < 0 || dutyCycle2 > 255)
    return -1;
    
  if(dutyCycle2 == 0) ledcAnalogWrite(pwmChannel2, 0);
  else ledcAnalogWrite(pwmChannel2, dutyCycle2);

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


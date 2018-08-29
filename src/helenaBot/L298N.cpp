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

#if(ESP_PLATFORM)
/* FK: Stolen from File/Examples/ESP32/AnalogOut/LEDCSoftwareFade */
// Arduino like analogWrite
// value has to be between 0 and valueMax
void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255)
{
  // calculate duty, 8191 from 2 ^ 13 - 1
  uint32_t duty = (8191 / valueMax) * min(value, valueMax);
  // write duty to LEDC
  ledcWrite(channel, duty);
}
#endif

DCMotor::DCMotor()
{
  enable_pin = -1;
  inA_pin = -1;
  inB_pin = -1;
  _motorState = STOP;
}

DCMotor::DCMotor(const int enable, const int inA, const int inB)
{
  enable_pin = enable;
  inA_pin = inA;
  inB_pin = inB;
  _motorState = STOP;
}

/* This is when we have an H-bridge with only one exposed pin
 *   to set the direction (that is, the inversion of the second
 *   channel (for the same bridge size) is implemented via
 *   hardware). */
DCMotor::DCMotor(const int enable, const int direction)
{
  enable_pin = enable;
  inA_pin = direction;
  inB_pin = -1;
  _motorState = STOP;
}

L298N::L298N(const DCMotor& motor)
{
  pinMode(motor.enable_pin, OUTPUT);
  digitalWrite(motor.enable_pin, LOW);

  pinMode(motor.inA_pin, OUTPUT);
  digitalWrite(motor.inA_pin, LOW);

  if(motor.inB_pin >= 0) {
    pinMode(motor.inB_pin, OUTPUT);
    digitalWrite(motor.inB_pin, LOW);
  }

#if(ESP_PLATFORM)
  pwmChannel1 = LEDC_CHANNEL_0;
  // Setup timer and attach timer to PWM pin
  ledcSetup(pwmChannel1, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
  ledcAttachPin(motor.enable_pin, pwmChannel1);
#endif
  
  /* Copy to internal object. */  
  _motor1 = motor;
  isMotor2Defined = false;
  pwmChannel2 = -1;
};

L298N::L298N(const DCMotor& motor1, const DCMotor& motor2)
{
  pinMode(motor1.enable_pin, OUTPUT);
  digitalWrite(motor1.enable_pin, LOW);

  pinMode(motor1.inA_pin, OUTPUT);
  digitalWrite(motor1.inA_pin, LOW);

  if(motor1.inB_pin >= 0) {
    pinMode(motor1.inB_pin, OUTPUT);
    digitalWrite(motor1.inB_pin, LOW);
  }

#if(ESP_PLATFORM)
  // Setup timer and attach timer to PWM pin
  pwmChannel1 = LEDC_CHANNEL_0;
  ledcSetup(pwmChannel1, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
  ledcAttachPin(motor1.enable_pin, pwmChannel1);
#endif

  /* Copy to internal object. */
  _motor1 = motor1;

  pinMode(motor2.enable_pin, OUTPUT);
  digitalWrite(motor2.enable_pin, LOW);

  pinMode(motor2.inA_pin, OUTPUT);
  digitalWrite(motor2.inA_pin, LOW);

  if(motor2.inB_pin >= 0) {
    pinMode(motor2.inB_pin, OUTPUT);
    digitalWrite(motor2.inB_pin, LOW);
  }

#if(ESP_PLATFORM)
  pwmChannel2 = LEDC_CHANNEL_1;
  // Setup timer and attach timer to PWM pin
  ledcSetup(pwmChannel2, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
  ledcAttachPin(motor2.enable_pin, pwmChannel2);
#endif

  /* Copy to internal object. */
  _motor2 = motor2;
  isMotor2Defined = true;
}

int L298N::setState(DCMotor& motor, const motorState state)
{
  if(motor.enable_pin < 0)
	  return -1;

  if(state == STOP)
    digitalWrite(motor.enable_pin, LOW);
  else if(state == RUN)
    digitalWrite(motor.enable_pin, HIGH);
  else
    return -1;

  motor._motorState = state;
  
  return 0;
}

int L298N::setState(const motorState state)
{
  int ret = setState(_motor1, state);
  if(!ret && isMotor2Defined)
    ret = setState(_motor2, state);

  return ret;
}

#if(!ESP_PLATFORM)
int L298N::setDutyCycle(DCMotor& motor, const unsigned int dutyCycle)
{
  /* Test for invalid parameters. */
  if(motor.enable_pin < 0 || dutyCycle > 255)
    return -1;

  if(dutyCycle == 0)
    return setState(motor, STOP);

  analogWrite(motor.enable_pin, dutyCycle);

  return 0;
}
#endif

int L298N::setDutyCycle(const unsigned int dutyCycle1, const unsigned int dutyCycle2)
{
#if(!ESP_PLATFORM)
  int ret = setDutyCycle(_motor1, dutyCycle1);
  if(!ret && isMotor2Defined)
    ret = setDutyCycle(_motor2, dutyCycle2);
#else

  /* MOTOR 1 */
  if(_motor1.enable_pin < 0 || dutyCycle1 > 255)
    return -1;

  if(dutyCycle1 == 0)
    setState(_motor1, STOP);
  else
    ledcAnalogWrite(pwmChannel1, dutyCycle1);

  /* MOTOR 2 */
  if(_motor2.enable_pin < 0 || dutyCycle2 > 255)
    return -1;
    
  if(dutyCycle2 == 0)
    setState(_motor2, STOP);
  else
    ledcAnalogWrite(pwmChannel2, dutyCycle2);

  return 0;
#endif
}

int L298N::setDutyCycle(const unsigned int dutyCycle)
{
#if(!ESP_PLATFORM)
  int ret = setDutyCycle(_motor1, dutyCycle);
  if(!ret && isMotor2Defined)
    ret = setDutyCycle(_motor2, dutyCycle);
  return ret;
#else
  return setDutyCycle(dutyCycle, dutyCycle);
#endif
}

int L298N::setDirection(DCMotor& motor, const motorDirection direction)
{
  if(motor.inA_pin < 0 && motor.inB_pin < 0)
	  return -1;

  if(direction == FW)
  {
    digitalWrite(motor.inA_pin, LOW);
    if(motor.inB_pin >= 0) 
      digitalWrite(motor.inB_pin, HIGH);
  }
  else if(direction == BW)
  {
    digitalWrite(motor.inA_pin, HIGH);
    if(motor.inB_pin >= 0) 
      digitalWrite(motor.inB_pin, LOW);
  }
  else
    return -1;

  motor._motorDirection = direction;

  return 0;
}

int L298N::setDirection(const motorDirection direction)
{
  int ret = setDirection(_motor1, direction);
  if(!ret && isMotor2Defined)
    ret = setDirection(_motor2, direction);

  return ret;
}


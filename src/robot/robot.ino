/*
 * HelenaBot
 * 
 * Felipe Kühne - fkuhne@gmail.com
 * 
 * 30/08/2018
 */

#include "L298N.h"

const int builtInLed = 13;
const int serialFrameSize = 11; /* Fixed size for a frame received through serial. */

const float deadBand = 10.0; /* Deadband region which keeps the robot stopped. */
unsigned long timeReceived = 0; /* Records when the last vlaid frame has been received. */
unsigned long timeoutBetweenCommand = 10000; /* Timeout between received commands, in milisseconds. */

const int enable1_pin = 3;
const int direction1_pin = 12;
const int enable2_pin = 11;
const int direction2_pin = 13;

DCMotor motor1(enable1_pin, direction1_pin);
DCMotor motor2(enable2_pin, direction2_pin);
L298N l298n(motor1, motor2);

/* This function was based on the example from
 * https://www.arduino.cc/en/Tutorial/ReadASCIIString. */
int waitCompleteSentence(int *digitalX, int *digitalY)
{
  /* Wait for an entire frame to be received. */
  if(Serial.available() < serialFrameSize)
    return -1;

  /* Once the frame is completed, we can parse it. */
  *digitalX = Serial.parseInt();
  *digitalY = Serial.parseInt();

  if(Serial.read() == '\n')
  {
    Serial.flush(); /* Clean up spurious data. */
    return 0;
  }

  return -1;
}

/* Read the digital Y and X values from the joystick and map them to linear
 * (tangential) speed and angular (rotational) speeds, respectively.
 *
 * digitalX and digitalY are on the range 0 ~ 1023. When the joystick is at
 * rest, their values are in the middle (1023/2), but can vary a little bit, so
 * let us consider a small deadband, and if the X/Y values are within it,
 * keep the robot still. We have to consider this for both axis.
 *
 * If we are outside the deadband area, we have to balance the X and Y
 * signals in order to generate linear and angular speeds, which will later be
 * transformed in PWM signals. The speeds will be mapped to percentage values,
 * from -100% to +100%, and then these values will be mapped to the logical
 * signals needed by the L298N H-bridge (enable signal (PWM) and inA/inB
 * (direction). This algorithm is based on
 *
 */
void applyControlSignals(int digitalX, int digitalY)
{
  const float deadbandLowerLimit = 1023.0 / 2.0 - deadBand;
  const float deadbandUpperLimit = 1023.0 / 2.0 + deadBand;

  /* If it's inside the deadband area, turn off the motors and return. */
  if(digitalX > deadbandLowerLimit &&
     digitalX < deadbandUpperLimit &&
     digitalY > deadbandLowerLimit &&
     digitalY < deadbandUpperLimit)
  {
    l298n.setState(STOP);
    return;
  }

  /* Here we are outside the deadband area. */

  float vPercentage = 0.0; /* Linear velocity in percentage level. */
  float wPercentage = 0.0; /* Angular velocity in percentage level. */

  /* Let's compute the angular velocity first. For the left, that is, between
   * 1023 and ((1023/2)+deadband), it is positive. For the right, between
   * ((1023/2)-deadband) and 0, it is negative. Let us then map this and
   * translate it to a percentage level, that is, translate from 1023~0 to
   * -100~100. */
  if(digitalX >= deadbandUpperLimit)
    wPercentage = map(digitalX, deadbandUpperLimit, 1023.0, 0.0, 100.0);

  else if(digitalX <= deadbandLowerLimit)
    wPercentage = map(digitalX, deadbandLowerLimit, 0.0, 0.0, -100.0);
  
  /* Let us now compute the linear velocity. The same scheme as above will
   * be applied, that is, translate from 0~1023 to -100~100. */
  if(digitalY >= deadbandUpperLimit)
    vPercentage = map(digitalY, deadbandUpperLimit, 1023.0, 0.0, 100.0);
 
  else if(digitalY <= deadbandLowerLimit)
    vPercentage = map(digitalY, deadbandLowerLimit, 0.0, 0.0, -100.0);
 
  /* Distribute the signals for left and right wheels, acoording to the
   * speeds. */
  int percentage1 = vPercentage + wPercentage;
  int percentage2 = vPercentage - wPercentage;

  /* Computes a scale factor. If any result exceeds 100% then adjust the scale
   * so that the result = 100% and use same scale value for other motor. */
  float maxPercentage = max(abs(percentage1), abs(percentage2));
  float scale = min(1.0,(100.0/maxPercentage));
  percentage1 *= scale;
  percentage2 *= scale;

  /* Duty Cycle for the motors. */
  int pwm1 = map(abs(percentage1), 0.0, 100.0, 0.0, 255.0);
  int pwm2 = map(abs(percentage2), 0.0, 100.0, 0.0, 255.0);

  Serial.print("pwm1 = "); Serial.print(pwm1);
  Serial.print(percentage1 > 0 ? " (FW)" : " (BW)");
  Serial.print(", pwm2 = "); Serial.print(pwm2);
  Serial.println(percentage2 > 0 ? " (FW)" : " (BW)");
  
  /* Finally, apply the control signals. */
  l298n.setDirection(motor1, (percentage1 > 0) ? FW : BW);
  l298n.setDirection(motor2, (percentage2 > 0) ? FW : BW);
  l298n.setDutyCycle(pwm1, pwm2);
}

void setup()
{
  l298n.setState(STOP);
  
  pinMode(builtInLed, OUTPUT);
  digitalWrite(builtInLed, HIGH);
  
  // Debug console
  Serial.begin(115200);
  Serial.println("Starting...");

    digitalWrite(builtInLed, LOW);  

  Serial.println("Exiting setup!");
}

void loop()
{

  /* Initialize with invalid data so that at each loop we can know if some
   *   valid has been received. */
  int digitalX = -1;
  int digitalY = -1;

  /* Check for valid data. */
  if(waitCompleteSentence(&digitalX, &digitalY) == 0)
  {
    /* Generate PWM values from joystick values. */
    applyControlSignals(digitalX, digitalY);
  }
}

/* Just a test function, in case you want to make sure the wirings are OK. */
void testBridgeAndMotors()
{
  l298n.setState(STOP);

  l298n.setDirection(FW);
  l298n.setState(RUN);
  delay(1000);

  l298n.setState(STOP);
  delay(1000);

  l298n.setDirection(BW);
  l298n.setState(RUN);
  delay(1000);

  l298n.setState(STOP);
  delay(1000);
}



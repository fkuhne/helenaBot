/*
 * Generated here:
 * https://examples.blynk.cc/?board=Arduino%20Nano&shield=HM10%20or%20HC08&example=Widgets%2FJoystickTwoAxis&auth=3e4d303c46524e0e96efca065804b527
 */

/*************************************************************
  Download latest Blynk library here:
    https://github.com/blynkkk/blynk-library/releases/latest

  Blynk is a platform with iOS and Android apps to control
  Arduino, Raspberry Pi and the likes over the Internet.
  You can easily build graphic interfaces for all your
  projects by simply dragging and dropping widgets.

    Downloads, docs, tutorials: http://www.blynk.cc
    Sketch generator:           http://examples.blynk.cc
    Blynk community:            http://community.blynk.cc
    Follow us:                  http://www.fb.com/blynkapp
                                http://twitter.com/blynk_app

  Blynk library is licensed under MIT license
  This example code is in public domain.

 *************************************************************
  Warning: Bluetooth support is in beta!

  You can receive x and y coords for joystick movement within App.

  App project setup:
    Two Axis Joystick on V1 in MERGE output mode.
    MERGE mode means device will receive both x and y within 1 message
 *************************************************************/

#define BLYNK_PRINT Serial /* Comment this out to disable prints and save space */

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include "L298N.h"

#if(ESP_PLATFORM)
  #define min(a,b) ((a)<(b)?(a):(b));
  #define max(a,b) ((a)>(b)?(a):(b));
#endif

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "2027d42aad2948a4b2a08f317886a9a8";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "GVT-A339"; //"DBLABDevices"; 
char pass[] = "0073162810"; //"@rdu!n0$";

SimpleTimer timer;
WidgetLED connectionLed(V2);
const int builtInLed = 13;

// Attach virtual serial terminal to Virtual Pin V0
//WidgetTerminal terminal(V0);

const float deadBand = 0.0; /* Deadband region which keeps the robot stopped. */
unsigned long timeReceived = 0; /* Records when the last vlaid frame has been received. */
unsigned long timeoutBetweenCommand = 10000; /* Timeout between received commands, in milisseconds. */

/* Pins used for motors:
 * Channel A: direction = D12
 *            PWM = D3
 *            Brake = D9
 *            Current sensor = A0
 * Channel B: direction = D13
 *            PWM = D11
 *            Brake = D8
 *            Current sensor = A1
 */
const int enable1_pin = 14; //3;
const int direction1_pin = 15; //12;
const int enable2_pin = 32; //11;
const int direction2_pin = 33; //13;

DCMotor motor1(enable1_pin, direction1_pin);
DCMotor motor2(enable2_pin, direction2_pin);
L298N l298n(motor1, motor2);

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

  /*terminal.print("pwm1 = "); terminal.print(pwm1);
  terminal.print(percentage1 > 0 ? " (FW)" : " (BW)");
  terminal.print(", pwm2 = "); terminal.print(pwm2);
  terminal.println(percentage2 > 0 ? " (FW)" : " (BW)");
  terminal.flush();*/

  motorDirection dir1, dir2;
  
  /* Finally, apply the control signals. */
  l298n.setDirection(motor1, (percentage1 > 0) ? FW : BW);
  l298n.setDirection(motor2, (percentage2 > 0) ? FW : BW);
  l298n.setDutyCycle(pwm1, pwm2);
}


/* Callback for terminal. */
/*BLYNK_WRITE(V0)
{
  if(String("en1 on")==param.asStr()) digitalWrite(14, HIGH);
  else if(String("en1 off")==param.asStr()) digitalWrite(14, LOW);
  else if(String("en2 on")==param.asStr()) digitalWrite(32, HIGH);
  else if(String("en2 off")==param.asStr()) digitalWrite(32, LOW);
  else if(String("dir1 fw")==param.asStr()) digitalWrite(15, HIGH);
  else if(String("dir1 bw")==param.asStr()) digitalWrite(15, LOW);
  else if(String("dir2 fw")==param.asStr()) digitalWrite(33, HIGH);
  else if(String("dir2 bw")==param.asStr()) digitalWrite(33, LOW);
}*/

/* Callback for joystick. */
BLYNK_WRITE(V1)
{
  /* Get the current time the command has been received. */
  timeReceived = millis();
  
  int x = param[0].asInt();
  int y = param[1].asInt();

  /*Serial.print("X = ");
  Serial.print(x);
  Serial.print("; Y = ");
  Serial.println(y);
  terminal.print("X = ");
  terminal.print(x);
  terminal.print("; Y = ");
  terminal.println(y);*/

  applyControlSignals(x, y);
}

void toggleConnectionLed()
{
  if(connectionLed.getValue()) connectionLed.off();
  else connectionLed.on();
}

void updateBatteryStatus()
{
  /* http://cuddletech.com/?p=1030, https://www.esp32.com/viewtopic.php?t=881 */
  
  float batt = analogRead(A13);
  /*batt /= 4095;
  batt *= 2;
  batt *= 3.3;
  batt *= 1.1;*/
  Blynk.virtualWrite(V0, batt);
  Serial.println("Battery level: ");
  Serial.print(batt);
}

void setup()
{
  l298n.setState(STOP);
  
  pinMode(builtInLed, OUTPUT);
  digitalWrite(builtInLed, HIGH);
  
  // Debug console
  Serial.begin(115200);
  Serial.println("Starting...");

  /* For cloud Blynk server. */
  Blynk.begin(auth, ssid, pass);
  /* For local server. */
  //Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8080);

  connectionLed.off();

  if(Blynk.connected())
  {
    digitalWrite(builtInLed, LOW);  
    Serial.println("Blynky connected.");
    //terminal.clear();
    //terminal.println("Blynky connected.");
  }

  timer.setInterval(2000, toggleConnectionLed);

  Serial.println("Exiting setup!");
  //terminal.println("Exiting setup!");
}

void loop()
{
  Blynk.run();
  timer.run();

  //testBridgeAndMotors();

  //unsigned long timeNow = millis();
  /* Compute the time interval between the last received frame
   *  and now. If higher than 1 second, turn off the motors. */
  /*if(timeNow - timeReceived > timeoutBetweenCommand)
  {
    //Serial.println("Communication timeout. Turning off motors.");
    timeReceived = millis();
    l298n.setState(STOP);
  }*/
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



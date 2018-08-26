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

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "3e4d303c46524e0e96efca065804b527";

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial
    
#include <BlynkSimpleSerialBLE.h>
#include <SoftwareSerial.h>

SoftwareSerial SerialBLE(10, 11); // RX, TX

// This function will be called every time Slider Widget
// in Blynk app writes values to the Virtual Pin 1
BLYNK_WRITE(V1)
{
  int x = param[0].asInt();
  int y = param[1].asInt();

  // Do something with x and y
  Serial.print("X = ");
  Serial.print(x);
  Serial.print("; Y = ");
  Serial.println(y);
}

void setup()
{
  // Debug console
  Serial.begin(9600);

  SerialBLE.begin(9600);
  Blynk.begin(SerialBLE, auth);

  Serial.println("Waiting for connections...");
}

void loop()
{
  Blynk.run();
}


/*
 * HelenaBot - code for the joystick (ESP32)
 * 
 * Felipe Kühne - fkuhne@gmail.com
 * 
 * 30/08/2018
 */

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
  
 *************************************************************/
 
//#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

HardwareSerial Serial1(2);

const int serialFrameSize = 11;
unsigned long timeReceived = 0; /* records when the last vlaid frame has been received. */
int receivedX = 512;
int receivedY = 512;
bool newData = false;

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "2027d42aad2948a4b2a08f317886a9a8";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Mini maker Faire";
char pass[] = "maker1234";

SimpleTimer timer;
WidgetLED connectionLed(V2);
const int builtInLed = 13;

/* Callback for joystick. */
BLYNK_WRITE(V1)
{
  receivedX = param[0].asInt();
  receivedY = param[1].asInt();
}

void toggleConnectionLed()
{
  if(connectionLed.getValue()) connectionLed.off();
  else connectionLed.on();
}

void setup()
{
  //Serial.begin(9600);
  pinMode(builtInLed, OUTPUT);
  digitalWrite(builtInLed, HIGH);
  
  /* For cloud Blynk server. */
  Blynk.begin(auth, ssid, pass);

  /* For local Blynk server - none of them worked. :( */
  //Blynk.begin(auth, "GVT-A339", "0073162810", IPAddress(169,254,68,130), 8080);
  //Blynk.begin(auth, "GVT-A339", "0073162810", IPAddress(192,168,25,179), 8080);
  
  connectionLed.off();

  if(Blynk.connected()) digitalWrite(builtInLed, LOW);

  timer.setInterval(1000, toggleConnectionLed);

  /* Serial port from pins 16 and 17. */
  Serial1.begin(9600);
  delay(1000);
}

void loop()
{
  Blynk.run();
  //timer.run();

  unsigned long timeNow = millis();
  if(timeNow - timeReceived > 100)
  {    
    timeReceived = millis();
    char sendBuffer[serialFrameSize+1];
    memset(sendBuffer, 0, serialFrameSize+1);
  
    /* Format the numbers with four digits, so we can fix the frame size that
     *   will be transmitted. */
    snprintf(sendBuffer, serialFrameSize, "%04d,%04d\n", receivedX, receivedY);
    Serial1.print(sendBuffer);
    //Serial1.flush();

    //Serial.print(sendBuffer);
  }
}


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

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

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
char ssid[] = "GVT-A339"; //"DBLABDevices"; 
char pass[] = "0073162810"; //"@rdu!n0$";

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
  pinMode(builtInLed, OUTPUT);
  digitalWrite(builtInLed, HIGH);
  
  /* For cloud Blynk server. */
  Blynk.begin(auth, ssid, pass);

  connectionLed.off();

  if(Blynk.connected()) digitalWrite(builtInLed, LOW);

  timer.setInterval(1000, toggleConnectionLed);

  Serial.begin(115200);
  delay(1000);
}

void loop()
{
  Blynk.run();
  timer.run();

  unsigned long timeNow = millis();
  if(timeNow - timeReceived > 200)
  {    
    timeReceived = millis();
    char sendBuffer[serialFrameSize] = "";
  
    /* Format the numbers with four digits, so we can fix the frame size that
     *   will be transmitted. */
    snprintf(sendBuffer, serialFrameSize, "%04d,%04d\n", receivedX, receivedY);
    Serial.print(sendBuffer);
    Serial.flush();
  }
}


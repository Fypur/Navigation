#include <Arduino.h>
#include "constants.h"
#include "wheel.h"
#include "wheelTest.h"


//needs to do 3 things
// 1 - receive speed orders (wheel1Speed, wheel2Speed...) from Raspberry PI
// 2 - Use PWM to send those speeds over to the wheels
// 3 - Get true wheel displacement through the encoders
// 4 - Send those displacements to the RPI for servoing (asservissement)

Wheel* wheelFrontLeft;
Wheel* wheelFrontRight;
Wheel* wheelBackRight;
Wheel* wheelBackLeft;
int speed = 255;

void receiveSpeedFromRPI(int);
void sendSpeedToWheels(int);

void setup() {

  Serial.begin(9600);
  
  wheelFrontLeft = new Wheel(frontLeftMotorSpeedPin, frontLeftMotorDirectionPin, false);
  wheelFrontRight = new Wheel(frontRightMotorSpeedPin, frontRightMotorDirectionPin, true);
  wheelBackRight = new Wheel(backRightMotorSpeedPin, backRightMotorDirectionPin, false);
  wheelBackLeft = new Wheel(backLeftMotorSpeedPin, backLeftMotorDirectionPin, false);

  //Important when using encoder
  /*attachInterrupt(digitalPinToInterrupt(encoderPin), []{ wheelFrontLeft->encoderPulse(); }, FALLING);
  attachInterrupt(digitalPinToInterrupt(encoderPin), []{ wheelFrontRight->encoderPulse(); }, FALLING);
  attachInterrupt(digitalPinToInterrupt(encoderPin), []{ wheelBackRight->encoderPulse(); }, FALLING);
  attachInterrupt(digitalPinToInterrupt(encoderPin), []{ wheelBackLeft->encoderPulse(); }, FALLING);*/
}

void loop() {
  int delayTime = 200;
  if (Serial.available()) {
    char val = Serial.read();
    if (val != -1)
    {
      switch (val)
      {
        case 'z':
          Serial.println("going forward");
          Forwards (wheelFrontLeft, wheelFrontRight, wheelBackLeft, wheelBackRight, speed);
          delay (delayTime);
          Stop(wheelFrontLeft, wheelFrontRight, wheelBackLeft, wheelBackRight);
          break;
        case 's':
          Serial.println("going backward");
          Backwards (wheelFrontLeft, wheelFrontRight, wheelBackLeft, wheelBackRight, speed);
          delay (delayTime);
          Stop(wheelFrontLeft, wheelFrontRight, wheelBackLeft, wheelBackRight);
          break;
        case 'q':
          Serial.println("going left");
          GoLeft (wheelFrontLeft, wheelFrontRight, wheelBackLeft, wheelBackRight, speed);
          delay (delayTime);
          Stop(wheelFrontLeft, wheelFrontRight, wheelBackLeft, wheelBackRight);
          break;
        case 'd':
          Serial.println("going right");
          GoRight (wheelFrontLeft, wheelFrontRight, wheelBackLeft, wheelBackRight, speed);
          delay (delayTime);
          Stop(wheelFrontLeft, wheelFrontRight, wheelBackLeft, wheelBackRight);
          break;
        case 'w':
          Serial.println("hello world!");
          break;
        case 'x':
          Serial.println("stopping");
          Stop(wheelFrontLeft, wheelFrontRight, wheelBackLeft, wheelBackRight);
          break;
      }
    }
    else Stop(wheelFrontLeft, wheelFrontRight, wheelBackLeft, wheelBackRight);
  }
}


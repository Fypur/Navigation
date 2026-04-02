#include "Arduino.h"
#include "wheel.h"
#include "wheelBenchmark.h"

//benchmark the RPM of a wheel with an encoder wired to the arduino at the given pins

float time1 = 0;
float time2 = 0;
volatile int encoderDownCount = 0;
int aimedSpeed = 40;
Wheel *benchmarkedWheel;

void wheelBenchmarkSetup(Wheel* benchmarkedWheel){
    benchmarkedWheel = benchmarkedWheel;
    pinMode(encoderAPin, INPUT_PULLUP);
    pinMode(encoderBPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoderBPin), pulse1, FALLING);
}

void wheelBenchmarkLoop()
{
    if (aimedSpeed >= 255)
    {
        Serial.println("Done");
        aimedSpeed = 0;
        benchmarkedWheel->SetSpeed(0);
    }
    else
        benchmarkedWheel->SetSpeed(aimedSpeed);
    // serialLoop();

    delay(500); // wait for wheel to speed up
    Encoder1(); // flush old data
    delay(1000);
    Encoder1();
    /*Serial.print(speed);
    Serial.print(',');*/
    Serial.print(rpm);
    Serial.print("\n");

    aimedSpeed += 2;
}

void pulse1()
{
    spinningForward = digitalRead(encoderAPin) == LOW;
    encoderDownCount++;
    time2 = millis();
}

void Encoder1()
{
    if (time2 > time1)
    {
        // Calculating the rpm of the motor
        rpm = 60000 * encoderDownCount / (time2 - time1);
        rpm = rpm / 234.3; // Gear ratio
        time1 = time2;
        encoderDownCount = 0;
    }
}
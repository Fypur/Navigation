#include "Arduino.h"
#include "wheel.h"

// benchmark the RPM of a wheel with an encoder wired to the arduino at the given pins

const int encoderAPin = 10;
const int encoderBPin = 2;

float rpm = 0;
bool spinningForward = false;


void wheelBenchmarkSetup(Wheel* benchmarkedWheel);
void wheelBenchmarkLoop();
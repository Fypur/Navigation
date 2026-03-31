#include "constants.h"
#include "orders.h"
#include "serial.h"
#include "wheel.h"
#include "wheelTest.h"
#include <Arduino.h>

#define BAUDRATE 115200

Wheel *wheelFrontLeft;
Wheel *wheelFrontRight;
Wheel *wheelBackRight;
Wheel *wheelBackLeft;
bool connectedToRaspi = false;

Order read_order() {
    return (Order)read_i8();
}

void write_order(Order order) {
    write_i8((int8_t)order);
}

void setup() {
    Serial.begin(BAUDRATE);

    wheelFrontLeft = new Wheel(frontLeftMotorSpeedPin, frontLeftMotorDirectionPin, false);
    wheelFrontRight = new Wheel(frontRightMotorSpeedPin, frontRightMotorDirectionPin,true);
    wheelBackRight = new Wheel(backRightMotorSpeedPin, backRightMotorDirectionPin, false);
    wheelBackLeft = new Wheel(backLeftMotorSpeedPin, backLeftMotorDirectionPin, false);

    pinMode(13, OUTPUT);
}

void loop() {
    if (Serial.available() <= 0){
        return;
    }

    // The first byte received is the instruction
    Order order_received = read_order();

    switch (order_received) {
        case Order::Hello: {
            if (!connectedToRaspi) {
                connectedToRaspi = true;
                write_order(Order::Hello);
            } else
                write_order(Order::AlreadyConnected);

            break;
        }
        case Order::AlreadyConnected: {
            connectedToRaspi = true;
            break;
        }
        case Order::WheelSpeeds: {
            int frontLeftWheelSpeed = (int)read_i16();
            int frontRightWheelSpeed = (int)read_i16();
            int backRightWheelSpeed = (int)read_i16();
            int backLeftWheelSpeed = (int)read_i16();

            wheelFrontLeft->SetSpeed(frontLeftWheelSpeed);
            wheelFrontRight->SetSpeed(frontRightWheelSpeed);
            wheelBackLeft->SetSpeed(backLeftWheelSpeed);
            wheelBackRight->SetSpeed(backRightWheelSpeed);
            break;
        }
    }
}
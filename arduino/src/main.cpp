#include "constants.h"
#include "orders.h"
#include "serial.h"
#include "wheel.h"
#include "wheelTest.h"
#include <Arduino.h>

#define BAUDRATE 9600

// needs to do 3 things
//  1 - receive speed orders (wheel1Speed, wheel2Speed...) from Raspberry PI
//  2 - Use PWM to send those
//  spc:\Users\amine\Downloads\serial-example\serial-example\slave\slave.h
//  c:\Users\amine\Downloads\serial-example\serial-example\slave\slave.inoeeds
//  over to the wheels 3 - Get true wheel displacement through the encoders 4 -
//  Send those displacements to the RPI for servoing (asservissement)

Wheel *wheelFrontLeft;
Wheel *wheelFrontRight;
Wheel *wheelBackRight;
Wheel *wheelBackLeft;
bool connectedToRaspi = false;
int speed = 255;

Order read_order() {
    return (Order)read_i8();
}

void write_order(Order order) {
    write_i8((int8_t)order);
}

void setup() {
    Serial.begin(BAUDRATE);

    wheelFrontLeft = new Wheel(frontLeftMotorSpeedPin, frontLeftMotorDirectionPin, false);
    wheelFrontRight = new Wheel(frontRightMotorSpeedPin, frontRightMotorDirectionPin, true);
    wheelBackRight = new Wheel(backRightMotorSpeedPin, backRightMotorDirectionPin, false);
    wheelBackLeft = new Wheel(backLeftMotorSpeedPin, backLeftMotorDirectionPin, false);
}

void loop() {
    if (Serial.available() <= 0)
        return;

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
        }
        case Order::WheelSpeeds: {
            int8_t frontLeftWheelSpeed = read_i8();
            int8_t frontRightWheelSpeed = read_i8();
            int8_t backLeftWheelSpeed = read_i8();
            int8_t backRightWheelSpeed = read_i8();

            wheelFrontLeft->SetSpeed(frontLeftWheelSpeed);
            wheelFrontRight->SetSpeed(frontRightWheelSpeed);
            wheelBackLeft->SetSpeed(backLeftWheelSpeed);
            wheelBackLeft->SetSpeed(backRightWheelSpeed);
        }
    }
}
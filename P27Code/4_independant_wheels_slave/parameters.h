#ifndef PARAMETERS_H
#define PARAMETERS_H

#define SERIAL_BAUD 115200


#define SPEED_FRONT_RIGHT 5
#define DIRECTION_FRONT_RIGHT 4

#define SPEED_FRONT_LEFT 6
#define DIRECTION_FRONT_LEFT 7

#define SPEED_REAR_RIGHT 11
#define DIRECTION_REAR_RIGHT 2

#define SPEED_REAR_LEFT 3
#define DIRECTION_REAR_LEFT 13



// Initial angle of the servomotor
#define INITIAL_THETA 110
// Min and max values for motors
#define THETA_MIN 60
#define THETA_MAX 150
#define SPEED_MAX 200
// If DEBUG is set to true, the arduino will send back all the received messages
#define DEBUG false
#define ADC_2 A0
#endif

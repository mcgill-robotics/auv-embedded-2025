#ifndef THRUSTER_CONTROL_H
#define THRUSTER_CONTROL_H

#include <Servo.h>

#define THRUSTER_1 2
#define THRUSTER_2 3
#define THRUSTER_3 4
#define THRUSTER_4 5
#define THRUSTER_5 6
#define THRUSTER_6 7
#define THRUSTER_7 8
#define THRUSTER_8 9

// Create an array of 8 thrusters
extern Servo thrusters[8];

// Define PWM signal (in microseconds) for each thruster
extern int16_t microseconds[8];

// Define the off command for all thrusters
extern const int16_t offCommand[8];

// Initializes the thrusters
void initThrusters();

// Updates the thrusters' PWM signal
void updateThrusters(const int16_t microseconds[8]);

#endif

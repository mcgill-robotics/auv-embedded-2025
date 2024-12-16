#include "ThrusterControl.h"

// Initialize thrusters
Servo thrusters[8];

// Default microseconds for each thruster (centered at 1500)
int16_t microseconds[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

// Off command to stop all thrusters (1500 microseconds)
const int16_t offCommand[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

// Initializes all thrusters by attaching them to the correct pins
void initThrusters() {
    thrusters[0].attach(THRUSTER_1);
    thrusters[1].attach(THRUSTER_2);
    thrusters[2].attach(THRUSTER_3);
    thrusters[3].attach(THRUSTER_4);
    thrusters[4].attach(THRUSTER_5);
    thrusters[5].attach(THRUSTER_6);
    thrusters[6].attach(THRUSTER_7);
    thrusters[7].attach(THRUSTER_8);

    updateThrusters(offCommand);  // Initialize all thrusters to the off state
}

// Updates all thrusters' PWM signal based on the provided array
void updateThrusters(const int16_t microseconds[8]) {
    for (int i = 0; i < 8; i++) {
        thrusters[i].writeMicroseconds(microseconds[i]);
    }
}

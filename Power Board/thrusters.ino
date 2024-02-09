/*

DO NOT FLASH TO TEENSY OR TO ARDUINO

*/

#include <Servo.h>

// defines all MCU pins
#define SRG_P_PIN 	2
#define SRG_S_PIN	3
#define SWY_BW_PIN 	4
#define SWY_ST_PIN 	5
#define HVE_BW_P_PIN 	6
#define HVE_BW_S_PIN 	7
#define HVE_ST_S_PIN 	8
#define HVE_ST_P_PIN 	9

#define MCU_KS 10
#define WATER_DETECTED 12

#define TEENSY_LED 13

#define TC_1 14
#define TC_2 15
#define TC_3 16
#define TC_4 17
#define TC_5 18
#define TC_6 19
#define TC_7 20
#define TC_8 21

#define VBAT1_SENSE 22
#define VBAT2_SENSE 23

// defines 8 thursters for initialization in an array
// should be replaced with definitions from ROS
#define SRG_P 	0
#define SRG_S	1
#define SWY_BW 	2
#define SWY_ST 	3
#define HVE_BW_P 	4
#define HVE_BW_S 	5
#define HVE_ST_S 	6
#define HVE_ST_P 	7

// creates array of 8 thrusters
Servo thrusters[8];

// signals to push to thrusters
uint16_t microseconds[] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
const uint16_t offCommand[] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

// creates array for 8 thruster currents
float currents[8];

// updates thrusters' pwm signals from array
void updateThrusters(const uint16_t microseconds[8]) {
	thrusters[SRG_P].writeMicroseconds(microseconds[SRG_P]);
	thrusters[SRG_S].writeMicroseconds(microseconds[SRG_S]);
	thrusters[SWY_BW].writeMicroseconds(microseconds[SWY_BW]);
	thrusters[SWY_ST].writeMicroseconds(microseconds[SWY_ST]);
	thrusters[HVE_BW_P].writeMicroseconds(microseconds[HVE_BW_P]);
	thrusters[HVE_BW_S].writeMicroseconds(microseconds[HVE_BW_S]);
	thrusters[HVE_ST_P].writeMicroseconds(microseconds[HVE_ST_P]);
	thrusters[HVE_ST_S].writeMicroseconds(microseconds[HVE_ST_S]);
}

// attaches and arms thrusters
void initThrusters() {
	thrusters[SRG_P].attach(SRG_P_PIN);
	thrusters[SRG_S].attach(SRG_S_PIN);
	thrusters[SWY_BW].attach(SWY_BW_PIN);
	thrusters[SWY_ST].attach(SWY_ST_PIN);
	thrusters[HVE_BW_P].attach(HVE_BW_P_PIN);
	thrusters[HVE_BW_S].attach(HVE_BW_S_PIN);
	thrusters[HVE_ST_S].attach(HVE_ST_S_PIN);
	thrusters[HVE_ST_P].attach(HVE_ST_P_PIN);

	updateThrusters(offCommand);
	delay(7000);
	// reamring works when system killed automatically at 2000
	// 7000 should be tested since it is more reliable based on bluerobotics
}

void killSystem() {
	digitalWrite(MCU_KS, HIGH);
	delay(100);
}

void powerSystem() {
	digitalWrite(MCU_KS, LOW);
	delay(100);
}

void waterInterrupt() {
	killSystem();
	while (true) {}
}

void senseCurrent(float currents[]) {
	currents[0] = ((analogRead(TC_1) * 3.3) / 1023) / 0.005;
	currents[1] = ((analogRead(TC_2) * 3.3) / 1023) / 0.005;
	currents[2] = ((analogRead(TC_3) * 3.3) / 1023) / 0.005;
    currents[3] = ((analogRead(TC_4) * 3.3) / 1023) / 0.005;
    currents[4] = ((analogRead(TC_5) * 3.3) / 1023) / 0.005;
    currents[5] = ((analogRead(TC_6) * 3.3) / 1023) / 0.005;
    currents[6] = ((analogRead(TC_7) * 3.3) / 1023) / 0.005;
    currents[7] = ((analogRead(TC_8) * 3.3) / 1023) / 0.005;
}

void setup() {
	//pinMode(WATER_DETECTED, INPUT_PULLUP);
	//pinMode(MCU_KS, OUTPUT);
	pinMode(TC_1, INPUT);
	pinMode(TC_2, INPUT);
	pinMode(TC_3, INPUT);
	pinMode(TC_4, INPUT);
	pinMode(TC_5, INPUT);
	pinMode(TC_6, INPUT);
	pinMode(TC_7, INPUT);
	pinMode(TC_8, INPUT);
	pinMode(VBAT1_SENSE, INPUT);
	pinMode(VBAT2_SENSE, INPUT);

	//attachInterrupt(digitalPinToInterrupt(WATER_DETECTED), waterInterrupt, RISING);

	initThrusters();
}

void loop() {
	updateThrusters(microseconds);
}

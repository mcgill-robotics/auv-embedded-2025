#include <Servo.h>
#include <ros.h>
#include <auv_msgs/ThrusterMicroseconds.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

// defines all MCU pins
#define FRONT_L_PIN 	2
#define FRONT_R_PIN	3
#define BACK_L_PIN 	4
#define BACK_R_PIN 	5
#define HEAVE_FRONT_L_PIN 	6
#define HEAVE_FRONT_R_PIN 	7
#define HEAVE_BACK_L_PIN 	8
#define HEAVE_BACK_R_PIN 	9

#define MCU_KS 10
// #define WATER_DETECTED 1

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

// defines 8 thursters for ROS subscribing
const uint8_t FRONT_L 	= auv_msgs::ThrusterMicroseconds::FRONT_LEFT;
const uint8_t FRONT_R 	= auv_msgs::ThrusterMicroseconds::FRONT_RIGHT;
const uint8_t BACK_L 	= auv_msgs::ThrusterMicroseconds::BACK_LEFT;
const uint8_t BACK_R 	= auv_msgs::ThrusterMicroseconds::BACK_RIGHT;
const uint8_t HEAVE_FRONT_L 	= auv_msgs::ThrusterMicroseconds::HEAVE_FRONT_LEFT;
const uint8_t HEAVE_FRONT_R 	= auv_msgs::ThrusterMicroseconds::HEAVE_FRONT_RIGHT;
const uint8_t HEAVE_BACK_L 	= auv_msgs::ThrusterMicroseconds::HEAVE_BACK_LEFT;
const uint8_t HEAVE_BACK_R 	= auv_msgs::ThrusterMicroseconds::HEAVE_BACK_RIGHT;

// defines 2 battery voltage sensing and 8 thruster current sensing messages for ROS advertising
std_msgs::Float32 batt1_voltage_msg;
std_msgs::Float32 batt2_voltage_msg;
std_msgs::Int32 thrust1_status_msg;
std_msgs::Int32 thrust2_status_msg;
std_msgs::Int32 thrust3_status_msg;
std_msgs::Int32 thrust4_status_msg;
std_msgs::Int32 thrust5_status_msg;
std_msgs::Int32 thrust6_status_msg;
std_msgs::Int32 thrust7_status_msg;
std_msgs::Int32 thrust8_status_msg;

// creates array of 8 thrusters
Servo thrusters[8];

// signals to push to thrusters
uint16_t microseconds[] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
const uint16_t offCommand[] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

// creates array for 8 thruster current sensing
double Tcurrents[8];

// creates array for 8 thruster status
int Sthrusters[8];

// creates array for 2 battery voltage sensing
float Bvoltages[2];

// updates thrusters' pwm signals from array
void updateThrusters(const uint16_t microseconds[8]) {
	thrusters[FRONT_L].writeMicroseconds(microseconds[FRONT_L]);
	thrusters[FRONT_R].writeMicroseconds(microseconds[FRONT_R]);
	thrusters[BACK_L].writeMicroseconds(microseconds[BACK_L]);
	thrusters[BACK_R].writeMicroseconds(microseconds[BACK_R]);
	thrusters[HEAVE_FRONT_L].writeMicroseconds(microseconds[HEAVE_FRONT_L]);
	thrusters[HEAVE_FRONT_R].writeMicroseconds(microseconds[HEAVE_FRONT_R]);
	thrusters[HEAVE_BACK_R].writeMicroseconds(microseconds[HEAVE_BACK_R]);
	thrusters[HEAVE_BACK_L].writeMicroseconds(microseconds[HEAVE_BACK_L]);
}

// updates microseconds array with values from ros
void commandCb(const auv_msgs::ThrusterMicroseconds& tc){
	memcpy(microseconds, tc.microseconds, 8*sizeof(uint16_t));
}

// attaches and arms thrusters
void initThrusters() {
	thrusters[FRONT_L].attach(FRONT_L_PIN);
	thrusters[FRONT_R].attach(FRONT_R_PIN);
	thrusters[BACK_L].attach(BACK_L_PIN);
	thrusters[BACK_R].attach(BACK_R_PIN);
	thrusters[HEAVE_FRONT_L].attach(HEAVE_FRONT_L_PIN);
	thrusters[HEAVE_FRONT_R].attach(HEAVE_FRONT_R_PIN);
	thrusters[HEAVE_BACK_L].attach(HEAVE_BACK_L_PIN);
	thrusters[HEAVE_BACK_R].attach(HEAVE_BACK_R_PIN);

	updateThrusters(offCommand);
}

// sets up ros publisher and subscriber nodes
ros::NodeHandle nh;
ros::Subscriber<auv_msgs::ThrusterMicroseconds> sub("/propulsion/microseconds", &commandCb);
ros::Publisher batt1_voltage("/batteries/voltage/1", &batt1_voltage_msg);
ros::Publisher batt2_voltage("/batteries/voltage/2", &batt2_voltage_msg);
ros::Publisher thrust1_status("/thrusters/status/1", &thrust1_status_msg);
ros::Publisher thrust2_status("/thrusters/status/2", &thrust2_status_msg);
ros::Publisher thrust3_status("/thrusters/status/3", &thrust3_status_msg);
ros::Publisher thrust4_status("/thrusters/status/4", &thrust4_status_msg);
ros::Publisher thrust5_status("/thrusters/status/5", &thrust5_status_msg);
ros::Publisher thrust6_status("/thrusters/status/6", &thrust6_status_msg);
ros::Publisher thrust7_status("/thrusters/status/7", &thrust7_status_msg);
ros::Publisher thrust8_status("/thrusters/status/8", &thrust8_status_msg);

// kills system by writing high to kill switch transistor
void killSystem() {
	digitalWrite(MCU_KS, HIGH);
	delay(100);
}

// powers on system by writing low to kill switch transistor
void powerSystem() {
	digitalWrite(MCU_KS, LOW);
	delay(100);
}

// permanently kills system by writing high to kill switch transistor and flashes led light
// void waterInterrupt() {
// 	delay(100);

// 	if (digitalRead(WATER_DETECTED)) {
// 		// killSystem();
// 		while (true) {
// 			digitalWrite(TEENSY_LED, HIGH);
// 			delay(500);
// 			digitalWrite(TEENSY_LED, LOW);
// 			delay(500);
// 		}
// 	}
// }

// senses currents of the 8 thrusters
void senseCurrent(double Tcurrents[]) {
	Tcurrents[0] = ((analogRead(TC_1) / 1024.0) * 3.3) / (0.005 * 50);
	Tcurrents[1] = ((analogRead(TC_2) / 1024.0) * 3.3) / (0.005 * 50);
	Tcurrents[2] = ((analogRead(TC_3) / 1024.0) * 3.3) / (0.005 * 50);
	Tcurrents[3] = ((analogRead(TC_4) / 1024.0) * 3.3) / (0.005 * 50);
	Tcurrents[4] = ((analogRead(TC_5) / 1024.0) * 3.3) / (0.005 * 50);
	Tcurrents[5] = ((analogRead(TC_6) / 1024.0) * 3.3) / (0.005 * 50);
	Tcurrents[6] = ((analogRead(TC_7) / 1024.0) * 3.3) / (0.005 * 50);
	Tcurrents[7] = ((analogRead(TC_8) / 1024.0) * 3.3) / (0.005 * 50);
}

void thrusterStatus(int Sthrusters[]) {
	for (int i = 0; i < 8; i++) {
		if (microseconds[i] == 1500) {
			Sthrusters[i] = 0;
		} else {
			Sthrusters[i] = 1;
		}
	}
}

// senses the voltages of the 2 batteries
void senseVoltage(float Bvoltages[]) {
	Bvoltages[0] = analogRead(VBAT1_SENSE) * (3.3 / 1024) * 1.6625 + 12.5;
	Bvoltages[1] = analogRead(VBAT2_SENSE) * (3.3 / 1024) * 1.6625 + 12.5;
}

// updates values sensed onto the ros nodes and publishes them
void publishVoltagesAndTStatus() {
	senseVoltage(Bvoltages);
	thrusterStatus(Sthrusters);

	batt1_voltage_msg.data = Bvoltages[0];
	batt2_voltage_msg.data = Bvoltages[1];
	thrust1_status_msg.data = Sthrusters[0];
	thrust2_status_msg.data = Sthrusters[1];
	thrust3_status_msg.data = Sthrusters[2];
	thrust4_status_msg.data = Sthrusters[3];
	thrust5_status_msg.data = Sthrusters[4];
	thrust6_status_msg.data = Sthrusters[5];
	thrust7_status_msg.data = Sthrusters[6];
	thrust8_status_msg.data = Sthrusters[7];
	
	batt1_voltage.publish( &batt1_voltage_msg );
	batt2_voltage.publish( &batt2_voltage_msg );
	thrust1_status.publish( &thrust1_status_msg );
	thrust2_status.publish( &thrust2_status_msg );
	thrust3_status.publish( &thrust3_status_msg );
	thrust4_status.publish( &thrust4_status_msg );
	thrust5_status.publish( &thrust5_status_msg );
	thrust6_status.publish( &thrust6_status_msg );
	thrust7_status.publish( &thrust7_status_msg );
	thrust8_status.publish( &thrust8_status_msg );
}

void setup() {
	initThrusters();

	pinMode(MCU_KS, OUTPUT);
	pinMode(TEENSY_LED, OUTPUT);
	// pinMode(WATER_DETECTED, INPUT_PULLUP);
	// attachInterrupt(digitalPinToInterrupt(WATER_DETECTED), waterInterrupt, RISING);
	// pinMode(TC_1, INPUT);
	// pinMode(TC_2, INPUT);
	// pinMode(TC_3, INPUT);
	// pinMode(TC_4, INPUT);
	// pinMode(TC_5, INPUT);
	// pinMode(TC_6, INPUT);
	// pinMode(TC_7, INPUT);
	// pinMode(TC_8, INPUT);
	pinMode(VBAT1_SENSE, INPUT);
	pinMode(VBAT2_SENSE, INPUT);

	nh.initNode();
	nh.subscribe(sub);
	nh.advertise(batt1_voltage);
	nh.advertise(batt2_voltage);
	nh.advertise(thrust1_status);
	nh.advertise(thrust2_status);
	nh.advertise(thrust3_status);
	nh.advertise(thrust4_status);
	nh.advertise(thrust5_status);
	nh.advertise(thrust6_status);
	nh.advertise(thrust7_status);
	nh.advertise(thrust8_status);
}

void loop() {
	updateThrusters(microseconds);

	publishVoltagesAndTStatus();
	
	nh.spinOnce();
	
	delay(10);
}

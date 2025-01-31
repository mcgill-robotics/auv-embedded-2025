/* 
Revision 1.0

SETUP FOR ROS COMMUNICATION WITH THE POWER BOARD
1) Initializes and connects thrusters, adc sensors, and temperature sensors
2) Sets up ros nodes for advertising and subscribing
3) Polls all sensors, publishes data, and updates thrusters every 10 ms
*/

#ifdef POWER_ROS1_H

#include "power_ros1_main.h"

#include <Arduino.h>

#include "ThrusterControl.h"
#include "adc_sensors.h"
#include "TMP36.h"

#include <Servo.h>

#include <ros.h>
#include <auv_msgs/ThrusterMicroseconds.h>
#include <std_msgs/Float32.h>

// defines thruster pins
#define BACK_L_PIN 2
#define HEAVE_BACK_L_PIN 3
#define HEAVE_FRONT_L_PIN 4
#define FRONT_L_PIN 5
#define FRONT_R_PIN 6
#define HEAVE_FRONT_R_PIN 7
#define HEAVE_BACK_R_PIN 8
#define BACK_R_PIN 9

#define LED_PIN 13

#define ENABLE_VOLTAGE_SENSE true
#define ENABLE_CURRENT_SENSE true

// creates ADCSensors and TMP36 sensor objects
ADCSensors adcSensors;
TMP36 temperatureSensor(23, 3.3);

// defines 8 thrusters for ROS subscribing
const uint8_t BACK_L = auv_msgs::ThrusterMicroseconds::BACK_LEFT;
const uint8_t HEAVE_BACK_L = auv_msgs::ThrusterMicroseconds::HEAVE_BACK_LEFT;
const uint8_t HEAVE_FRONT_L = auv_msgs::ThrusterMicroseconds::HEAVE_FRONT_LEFT;
const uint8_t FRONT_L = auv_msgs::ThrusterMicroseconds::FRONT_LEFT;
const uint8_t FRONT_R = auv_msgs::ThrusterMicroseconds::FRONT_RIGHT;
const uint8_t HEAVE_FRONT_R = auv_msgs::ThrusterMicroseconds::HEAVE_FRONT_RIGHT;
const uint8_t HEAVE_BACK_R = auv_msgs::ThrusterMicroseconds::HEAVE_BACK_RIGHT;
const uint8_t BACK_R = auv_msgs::ThrusterMicroseconds::BACK_RIGHT;

// defines 2 battery voltage sensing for ROS advertising
std_msgs::Float32 batt1_voltage_msg;
std_msgs::Float32 batt2_voltage_msg;

// defines 8 thruster current sensing for ROS advertising
std_msgs::Float32 thruster1_current_msg;
std_msgs::Float32 thruster2_current_msg;
std_msgs::Float32 thruster3_current_msg;
std_msgs::Float32 thruster4_current_msg;
std_msgs::Float32 thruster5_current_msg;
std_msgs::Float32 thruster6_current_msg;
std_msgs::Float32 thruster7_current_msg;
std_msgs::Float32 thruster8_current_msg;

// defines board and teensy temperature sensing for ROS advertising
std_msgs::Float32 board_temperature_msg;
std_msgs::Float32 teensy_temperature_msg;

// creates array of 8 thrusters
Servo thrusters[8];

// signals to push to thrusters
uint16_t microseconds[] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
const uint16_t offCommand[] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

float Bvoltages[2]; // array for 2 battery voltage sensing
float Tcurrents[8]; // array for 8 thrusters current sensing
float boardTemperature;
float teensyTemperature;

// updates thrusters' pwm signals from array
void updateThrusters(const uint16_t microseconds[8]) {
    thrusters[BACK_L].writeMicroseconds(microseconds[BACK_L]);
    thrusters[HEAVE_BACK_L].writeMicroseconds(microseconds[HEAVE_BACK_L]);
    thrusters[HEAVE_FRONT_L].writeMicroseconds(microseconds[HEAVE_FRONT_L]);
    thrusters[FRONT_L].writeMicroseconds(microseconds[FRONT_L]);
    thrusters[FRONT_R].writeMicroseconds(microseconds[FRONT_R]);
    thrusters[HEAVE_FRONT_R].writeMicroseconds(microseconds[HEAVE_FRONT_R]);
    thrusters[BACK_R].writeMicroseconds(microseconds[BACK_R]);
    thrusters[HEAVE_BACK_R].writeMicroseconds(microseconds[HEAVE_BACK_R]);
}

// updates microseconds array with values from ros
void commandCb(const auv_msgs::ThrusterMicroseconds &tc) {
    memcpy(microseconds, tc.microseconds, 8 * sizeof(uint16_t));
}

// attaches and arms thrusters
void initThrusters() {
    thrusters[BACK_L].attach(BACK_L_PIN);
    thrusters[HEAVE_BACK_L].attach(HEAVE_BACK_L_PIN);
    thrusters[HEAVE_FRONT_L].attach(HEAVE_FRONT_L_PIN);
    thrusters[FRONT_L].attach(FRONT_L_PIN);
    thrusters[FRONT_R].attach(FRONT_R_PIN);
    thrusters[HEAVE_FRONT_R].attach(HEAVE_FRONT_R_PIN);
    thrusters[HEAVE_BACK_R].attach(HEAVE_BACK_R_PIN);
    thrusters[BACK_R].attach(BACK_R_PIN);

    updateThrusters(offCommand);
}

// sets up ros publisher and subscriber nodes
ros::NodeHandle nh;
ros::Subscriber<auv_msgs::ThrusterMicroseconds> sub("/propulsion/microseconds", &commandCb);
ros::Publisher batt1_voltage("/power/batteries/voltage/1", &batt1_voltage_msg);
ros::Publisher batt2_voltage("/power/batteries/voltage/2", &batt2_voltage_msg);
ros::Publisher thruster1_current("/power/thrusters/current/1", &thruster1_current_msg);
ros::Publisher thruster2_current("/power/thrusters/current/2", &thruster2_current_msg);
ros::Publisher thruster3_current("/power/thrusters/current/3", &thruster3_current_msg);
ros::Publisher thruster4_current("/power/thrusters/current/4", &thruster4_current_msg);
ros::Publisher thruster5_current("/power/thrusters/current/5", &thruster5_current_msg);
ros::Publisher thruster6_current("/power/thrusters/current/6", &thruster6_current_msg);
ros::Publisher thruster7_current("/power/thrusters/current/7", &thruster7_current_msg);
ros::Publisher thruster8_current("/power/thrusters/current/8", &thruster8_current_msg);
ros::Publisher board_temperature("/power/board/temperature", &board_temperature_msg);
ros::Publisher teensy_temperature("/power/teensy/temperature", &teensy_temperature_msg);

// senses the battery voltages, thruster currents, and board and teensy temperatures
void senseData() {
    float* voltagePtr = adcSensors.senseVoltage();
    float* currentPtr = adcSensors.senseCurrent();

    // Copy the values into the fixed-size arrays
    for (int i = 0; i < 2; ++i) {
        Bvoltages[i] = voltagePtr[i];
    }

    for (int i = 0; i < 8; ++i) {
        Tcurrents[i] = currentPtr[i];
    }

    boardTemperature = temperatureSensor.readTemperature();
    teensyTemperature = tempmonGetTemp();
}

// updates values sensed onto the ros nodes and publishes them
void publishData() {
    senseData();

    batt1_voltage_msg.data = Bvoltages[0];
    batt2_voltage_msg.data = Bvoltages[1];

    batt1_voltage.publish(&batt1_voltage_msg);
    batt2_voltage.publish(&batt2_voltage_msg);

    thruster1_current_msg.data = Tcurrents[0];
    thruster2_current_msg.data = Tcurrents[1];
    thruster3_current_msg.data = Tcurrents[2];
    thruster4_current_msg.data = Tcurrents[3];
    thruster5_current_msg.data = Tcurrents[4];
    thruster6_current_msg.data = Tcurrents[5];
    thruster7_current_msg.data = Tcurrents[6];
    thruster8_current_msg.data = Tcurrents[7];

    thruster1_current.publish(&thruster1_current_msg);
    thruster2_current.publish(&thruster2_current_msg);
    thruster3_current.publish(&thruster3_current_msg);
    thruster4_current.publish(&thruster4_current_msg);
    thruster5_current.publish(&thruster5_current_msg);
    thruster6_current.publish(&thruster6_current_msg);
    thruster7_current.publish(&thruster7_current_msg);
    thruster8_current.publish(&thruster8_current_msg);

    board_temperature_msg.data = boardTemperature;
    teensy_temperature_msg.data = teensyTemperature;

    board_temperature.publish(&board_temperature_msg);
    teensy_temperature.publish(&teensy_temperature_msg);
}

// setup all thrusters and sensors and setup node handler for subscribing and advertising
void power_ros1_setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); // turn on LED_PIN

    initThrusters();

    adcSensors.begin(ENABLE_VOLTAGE_SENSE, ENABLE_CURRENT_SENSE, &Wire1);
    temperatureSensor.begin();

    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(batt1_voltage);
    nh.advertise(batt2_voltage);
    nh.advertise(thruster1_current);
    nh.advertise(thruster2_current);
    nh.advertise(thruster3_current);
    nh.advertise(thruster4_current);
    nh.advertise(thruster5_current);
    nh.advertise(thruster6_current);
    nh.advertise(thruster7_current);
    nh.advertise(thruster8_current);
    nh.advertise(board_temperature);
    nh.advertise(teensy_temperature);
}

void power_ros1_loop() {
    updateThrusters(microseconds); 

    publishData();

    nh.spinOnce();

    delay(10);
}

#endif

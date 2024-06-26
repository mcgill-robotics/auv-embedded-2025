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

// defines voltage sensing pins
#define VBAT1_SENSE 22
#define VBAT2_SENSE 23

// defines 8 thursters for ROS subscribing
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

// creates array of 8 thrusters
Servo thrusters[8];

// signals to push to thrusters
uint16_t microseconds[] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
const uint16_t offCommand[] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

// creates array for 2 battery voltage sensing
float Bvoltages[2];

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
ros::Publisher batt1_voltage("/display/batteries/voltage/1", &batt1_voltage_msg);
ros::Publisher batt2_voltage("/display/batteries/voltage/2", &batt2_voltage_msg);

// senses the voltages of the 2 batteries
void senseVoltage(float Bvoltages[]) {
  Bvoltages[0] = analogRead(VBAT1_SENSE) * (3.3 / 1024) * 1.6625 + 12.5;
  Bvoltages[1] = analogRead(VBAT2_SENSE) * (3.3 / 1024) * 1.6625 + 12.5;
}

// updates values sensed onto the ros nodes and publishes them
void publishVoltages() {
  senseVoltage(Bvoltages);

  batt1_voltage_msg.data = Bvoltages[0];
  batt2_voltage_msg.data = Bvoltages[1];

  batt1_voltage.publish(&batt1_voltage_msg);
  batt2_voltage.publish(&batt2_voltage_msg);
}

void setup() {
  initThrusters();

  pinMode(VBAT1_SENSE, INPUT);
  pinMode(VBAT2_SENSE, INPUT);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(batt1_voltage);
  nh.advertise(batt2_voltage);
}

void loop() {
  updateThrusters(microseconds);

  publishVoltages();

  nh.spinOnce();

  delay(10);
}
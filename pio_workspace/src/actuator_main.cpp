#ifdef ACTUATOR_H

#include "actuator_main.h"

#include <Arduino.h>

#include <Servo.h>

#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>

// servo pin definition
const int SERVO_PIN = 9;

// create node handle
ros::NodeHandle nh;

// create grabber servo object
Servo grabberServo;

// function prototypes
void servoPosition_CB(const std_msgs::UInt16& msg);
void servoSweep_CB(const std_msgs::Bool& msg);

// subscribe to topics
ros::Subscriber<std_msgs::UInt16> grabberPositionSub("servo/position", &servoPosition_CB);
ros::Subscriber<std_msgs::Bool> grabberSweepSub("servo/sweep", &servoSweep_CB);

// setup function
void actuator_setup() {
    // pwm teensy pin connected to grabber
  pinMode(SERVO_PIN, OUTPUT);
    // attach servo object to teensy pin and check for success
  if (grabberServo.attach(SERVO_PIN)) {
    // servo attached successfully
  } else {
    // servo attachment failed - could add LED blink or error handling here
  }
    // initialize ros node
  nh.initNode();
    // subscribe to ros topics
  nh.subscribe(grabberPositionSub);
  nh.subscribe(grabberSweepSub);
}

// check for new messages with 1ms delay
void actuator_loop() {
  nh.spinOnce();
  delay(1);
}

// move grabber servo to position specified by ros message
void servoPosition_CB(const std_msgs::UInt16& msg) {
  // constrain servo position to valid range (0-180 degrees)
  int position = constrain(msg.data, 0, 180);
  grabberServo.write(position);
}

// sweep servo based on boolean message
void servoSweep_CB(const std_msgs::Bool& msg) {
  if (msg.data) {
    // sweep from 0 to 180 degrees
    for (int pos = 0; pos <= 180; pos += 1) {
      grabberServo.write(pos);
      delay(15);
      // check for new messages during sweep to maintain some responsiveness
      nh.spinOnce();
    }
  }
  else {
    // sweep from 180 to 0 degrees
    for (int pos = 180; pos >= 0; pos -= 1) {
      grabberServo.write(pos);
      delay(15);
      // check for new messages during sweep to maintain some responsiveness
      nh.spinOnce();
    }
  }
}

#endif

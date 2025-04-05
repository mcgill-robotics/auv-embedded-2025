#ifdef ACTUATOR_H

#include "actuator_main.h"
#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle nh;

Servo myServo;

void servoCallback(const std_msgs::UInt16& msg) {
  int pos = constrain(msg.data, 0, 180);  // Limit angle between 0 and 180
  myServo.write(pos);
}

ros::Subscriber<std_msgs::UInt16> sub("servo_position", &servoCallback);

void actuator_setup() {
  myServo.attach(9);  // Connect servo signal wire to pin 9
  nh.initNode();
  nh.subscribe(sub);
}

void actuator_loop() {
  nh.spinOnce();
  delay(10);
}

#endif
#ifdef ACTUATOR_H

#include "actuator_main.h"
#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh;

Servo myServo;

void servoPosition_CB(const std_msgs::UInt16& msg) {
  myServo.write(msg.data);
}

void servoSweep_CB(const std_msgs::Bool& msg) {
  if (msg.data) {
    for (int pos = 0; pos <= 180; pos += 1) {                                 
      myServo.write(pos);
      delay(15);
    } 
  }
  else {
    for (int pos = 180; pos >= 0; pos -= 1) {                                
      myServo.write(pos);
      delay(15);
    } 
}
}

ros::Subscriber<std_msgs::UInt16> sub("servo/position", &servoPosition_CB);
ros::Subscriber<std_msgs::Bool> sub2("servo/sweep", &servoSweep_CB);

void actuator_setup() {
  pinMode(9, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub2);
  myServo.attach(9);
}

void actuator_loop() {
  nh.spinOnce();
  delay(1);
}

#endif
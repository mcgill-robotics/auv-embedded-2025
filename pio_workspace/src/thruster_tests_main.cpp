#ifdef THRUSTER_TESTS_H

#include "thruster_tests_main.h"

#include <Arduino.h>

#include <ros.h>
#include <auv_msgs/ThrusterMicroseconds.h>
#include <std_msgs/Float32.h>

#define LED_PIN 13

std_msgs::Float32 thruster_force_msg;
float thrusterForce = -1;

uint16_t microseconds[] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
void commandCb(const auv_msgs::ThrusterMicroseconds &tc) {
    memcpy(microseconds, tc.microseconds, 8 * sizeof(uint16_t));
}

ros::NodeHandle nh;
ros::Subscriber<auv_msgs::ThrusterMicroseconds> sub("/propulsion/microseconds", &commandCb);
ros::Publisher thruster_force("/thruster/force", &thruster_force_msg);

void senseData() {
    thrusterForce++;
}

void publishData() {
    senseData();
    thruster_force_msg.data = thrusterForce;
    thruster_force.publish(&thruster_force_msg);
}

void thruster_tests_setup() {
    pinMode(2, INPUT);
    pinMode(3, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(thruster_force);
}

void thruster_tests_loop() {
    if (digitalRead(2) == HIGH) {
        digitalWrite(3, HIGH);
        digitalWrite(4, LOW);
    } else {
        digitalWrite(3, LOW);
        digitalWrite(4, HIGH);
    }
    publishData();
    nh.spinOnce();
    delay(10);
}

#endif

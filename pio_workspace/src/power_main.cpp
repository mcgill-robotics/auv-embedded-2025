#include "power_main.h"

#include <Servo.h>

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int16_multi_array.h>

#define THRUSTER_1 2
#define THRUSTER_2 3
#define THRUSTER_3 4
#define THRUSTER_4 5
#define THRUSTER_5 6
#define THRUSTER_6 7
#define THRUSTER_7 8
#define THRUSTER_8 9

rcl_subscription_t subscriber;
std_msgs__msg__Int16MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  free(msg.data.data);

  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// creates array of 8 thrusters
Servo thrusters[8];

// signals to push to thrusters
int16_t microseconds[] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
const int16_t offCommand[] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

void subscription_callback(const void * msgin)
{
    const std_msgs__msg__Int16MultiArray * msg = (const std_msgs__msg__Int16MultiArray *)msgin;

    // Ensure we don't exceed the size of the `microseconds` array
    for (size_t i = 0; i < 8 && i < msg->data.size; i++) {
        microseconds[i] = msg->data.data[i]; // Access the data correctly
    }
}

// updates thrusters' pwm signals from array
void updateThrusters(const int16_t microseconds[8]) {
	thrusters[0].writeMicroseconds(microseconds[0]);
	thrusters[1].writeMicroseconds(microseconds[1]);
	thrusters[2].writeMicroseconds(microseconds[2]);
	thrusters[3].writeMicroseconds(microseconds[3]);
	thrusters[4].writeMicroseconds(microseconds[4]);
	thrusters[5].writeMicroseconds(microseconds[5]);
	thrusters[6].writeMicroseconds(microseconds[6]);
	thrusters[7].writeMicroseconds(microseconds[7]);
}

void initThrusters() {
	thrusters[0].attach(THRUSTER_1);
	thrusters[1].attach(THRUSTER_2);
	thrusters[2].attach(THRUSTER_3);
	thrusters[3].attach(THRUSTER_4);
	thrusters[4].attach(THRUSTER_5);
	thrusters[5].attach(THRUSTER_6);
	thrusters[6].attach(THRUSTER_7);
	thrusters[7].attach(THRUSTER_8);

	updateThrusters(offCommand);
}

void power_setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  initThrusters();
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "power_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
    "/propulsion/microseconds"));

    msg.data.data = (int16_t *)malloc(8 * sizeof(int16_t));
    msg.data.size = 8;
    msg.data.capacity = 8;

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void power_loop() {
  updateThrusters(microseconds);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

#include <micro_ros_arduino.h>

#include <Arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int16_multi_array.h>

#define LED_PIN 13

rcl_subscription_t propulsion_microseconds_subscriber;
std_msgs__msg__Int16MultiArray propulsion_microseconds_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

extern int16_t microseconds[8];

void micro_ros_init();
void spin_micro_ros();
void error_loop();

void propulsion_microseconds_subscription_callback(const void * msgin);

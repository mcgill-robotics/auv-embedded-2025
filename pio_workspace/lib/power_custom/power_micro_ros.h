#include <micro_ros_arduino.h>

#include <Arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/float32.h>

#define LED_PIN 13

extern rcl_subscription_t propulsion_microseconds_subscriber;
extern std_msgs__msg__Int16MultiArray propulsion_microseconds_msg;

extern rcl_publisher_t power_board_temperature_publisher;
extern std_msgs__msg__Float32 power_board_temperature_msg;

extern rclc_executor_t executor;
extern rclc_support_t support;
extern rcl_allocator_t allocator;
extern rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

extern int16_t microseconds[8];

void micro_ros_init();
void spin_micro_ros();
void error_loop();

void power_board_temperature_publish(float power_board_temperature);

void propulsion_microseconds_subscription_callback(const void * msgin);

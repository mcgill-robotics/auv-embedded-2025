#ifdef POWER_H

#include "power_main.h"
#include <Arduino.h>

/*
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int16_multi_array.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rcl_publisher_t array_publisher;
std_msgs__msg__Int16MultiArray array_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    RCSOFTCHECK(rcl_publish(&array_publisher, &array_msg, NULL));
    msg.data++;

    array_msg.data.data[0]++;
    array_msg.data.data[1]++;
    array_msg.data.data[2]++;
    array_msg.data.data[3]++;
    array_msg.data.data[4]++;
    array_msg.data.data[5]++;
    array_msg.data.data[6]++;
    array_msg.data.data[7]++;
  }
}

void power_setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  array_msg.data.size = 8;
  array_msg.data.capacity = 8;
  array_msg.data.data = (int16_t*)malloc(array_msg.data.capacity * sizeof(int16_t));

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_platformio_node_publisher"));

  RCCHECK(rclc_publisher_init_default(
    &array_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
    "array_publisher"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data = 0;
  array_msg.data.data[0] = 0;
  array_msg.data.data[1] = 1;
  array_msg.data.data[2] = 2;
  array_msg.data.data[3] = 3;
  array_msg.data.data[4] = 4;
  array_msg.data.data[5] = 5;
  array_msg.data.data[6] = 6;
  array_msg.data.data[7] = 7;
}

void power_loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

*/

/*
#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int16_multi_array.h>


rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;

rcl_subscription_t array_subscriber;
std_msgs__msg__Int16MultiArray array_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH);  
}

void array_subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int16MultiArray * msg = (const std_msgs__msg__Int16MultiArray *)msgin;
  digitalWrite(2, msg->data.data[0] == 0 ? LOW : HIGH);
  digitalWrite(3, msg->data.data[1] == 0 ? LOW : HIGH);
  digitalWrite(4, msg->data.data[2] == 0 ? LOW : HIGH);
  digitalWrite(5, msg->data.data[3] == 0 ? LOW : HIGH);
  digitalWrite(6, msg->data.data[4] == 0 ? LOW : HIGH);
  digitalWrite(7, msg->data.data[5] == 0 ? LOW : HIGH);
  digitalWrite(8, msg->data.data[6] == 0 ? LOW : HIGH);
  digitalWrite(9, msg->data.data[7] == 0 ? LOW : HIGH);

}

void power_setup() {
  pinMode(13, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);

  digitalWrite(13, HIGH);
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  array_msg.data.size = 8;
  array_msg.data.capacity = 8;
  array_msg.data.data = (int16_t*)malloc(array_msg.data.capacity * sizeof(int16_t));

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_arduino_subscriber"));

  RCCHECK(rclc_subscription_init_default(
    &array_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
    "array_subscriber"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator)); // this errors out if not 2, basically it is number of subscribers plus a bunch of things
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &array_subscriber, &array_msg, &array_subscription_callback, ON_NEW_DATA));
}

void power_loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

*/

/*

#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int16_multi_array.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

#define LED_PIN 13

rcl_subscription_t array_subscriber;
std_msgs__msg__Int16MultiArray array_msg_subscription;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rcl_publisher_t array_publisher;
std_msgs__msg__Int16MultiArray array_msg_publishing;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH);  
}

void array_subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int16MultiArray * msg = (const std_msgs__msg__Int16MultiArray *)msgin;
  digitalWrite(2, msg->data.data[0] == 0 ? LOW : HIGH);
  digitalWrite(3, msg->data.data[1] == 0 ? LOW : HIGH);
  digitalWrite(4, msg->data.data[2] == 0 ? LOW : HIGH);
  digitalWrite(5, msg->data.data[3] == 0 ? LOW : HIGH);
  digitalWrite(6, msg->data.data[4] == 0 ? LOW : HIGH);
  digitalWrite(7, msg->data.data[5] == 0 ? LOW : HIGH);
  digitalWrite(8, msg->data.data[6] == 0 ? LOW : HIGH);
  digitalWrite(9, msg->data.data[7] == 0 ? LOW : HIGH);

}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    RCSOFTCHECK(rcl_publish(&array_publisher, &array_msg_publishing, NULL));
    msg.data++;

    array_msg_publishing.data.data[0]++;
    array_msg_publishing.data.data[1]++;
    array_msg_publishing.data.data[2]++;
    array_msg_publishing.data.data[3]++;
    array_msg_publishing.data.data[4]++;
    array_msg_publishing.data.data[5]++;
    array_msg_publishing.data.data[6]++;
    array_msg_publishing.data.data[7]++;
  }
}

void power_setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);

  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  array_msg_publishing.data.size = 8;
  array_msg_publishing.data.capacity = 8;
  array_msg_publishing.data.data = (int16_t*)malloc(array_msg_publishing.data.capacity * sizeof(int16_t));

  array_msg_subscription.data.size = 8;
  array_msg_subscription.data.capacity = 8;
  array_msg_subscription.data.data = (int16_t*)malloc(array_msg_subscription.data.capacity * sizeof(int16_t));

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_platformio_node_publisher"));

  RCCHECK(rclc_publisher_init_default(
    &array_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
    "array_publisher"));

  RCCHECK(rclc_subscription_init_default(
    &array_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
    "array_subscriber"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 10, &allocator)); // number arbitrarily set, idk what is the correct on yet, trial and error later on
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &array_subscriber, &array_msg_subscription, &array_subscription_callback, ON_NEW_DATA));

  msg.data = 0;
  array_msg_publishing.data.data[0] = 0;
  array_msg_publishing.data.data[1] = 1;
  array_msg_publishing.data.data[2] = 2;
  array_msg_publishing.data.data[3] = 3;
  array_msg_publishing.data.data[4] = 4;
  array_msg_publishing.data.data[5] = 5;
  array_msg_publishing.data.data[6] = 6;
  array_msg_publishing.data.data[7] = 7;
}

void power_loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

// ros2 topic pub /array_subscriber std_msgs/msg/Int16MultiArray "data: [1, 1, 0, 0, 0, 0, 0, 0]"

*/




#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

#define LED_PIN 13

bool micro_ros_init_successful;

rcl_subscription_t propulsion_microseconds_subscriber;
std_msgs__msg__Int16MultiArray propulsion_microseconds_msg;

rcl_publisher_t power_batteries_voltage_publisher;
std_msgs__msg__Float32MultiArray power_batteries_voltage_msg;

rcl_publisher_t power_thrusters_current_publisher;
std_msgs__msg__Float32MultiArray power_thrusters_current_msg;

rcl_publisher_t power_board_temperature_publisher;
std_msgs__msg__Float32 power_board_temperature_msg;

rcl_publisher_t power_teensy_temperature_publisher;
std_msgs__msg__Float32 power_teensy_temperature_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

void error_loop() {
    int error = 0;
    while (error < 10) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(100);

        error++;
    }
    digitalWrite(LED_BUILTIN, HIGH);
}

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

void propulsion_microseconds_callback(const void * msgin)
{  
  const std_msgs__msg__Int16MultiArray * msg = (const std_msgs__msg__Int16MultiArray *)msgin;

  /*
  digitalWrite(2, msg->data.data[0] == 0 ? LOW : HIGH);
  digitalWrite(3, msg->data.data[1] == 0 ? LOW : HIGH);
  digitalWrite(4, msg->data.data[2] == 0 ? LOW : HIGH);
  digitalWrite(5, msg->data.data[3] == 0 ? LOW : HIGH);
  digitalWrite(6, msg->data.data[4] == 0 ? LOW : HIGH);
  digitalWrite(7, msg->data.data[5] == 0 ? LOW : HIGH);
  digitalWrite(8, msg->data.data[6] == 0 ? LOW : HIGH);
  digitalWrite(9, msg->data.data[7] == 0 ? LOW : HIGH);
  */
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&power_batteries_voltage_publisher, &power_batteries_voltage_msg, NULL));
    RCSOFTCHECK(rcl_publish(&power_thrusters_current_publisher, &power_thrusters_current_msg, NULL));
    RCSOFTCHECK(rcl_publish(&power_board_temperature_publisher, &power_board_temperature_msg, NULL));
    RCSOFTCHECK(rcl_publish(&power_teensy_temperature_publisher, &power_teensy_temperature_msg, NULL));

    // can add logic to messages here
  }
}

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "power_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &propulsion_microseconds_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
    "/propulsion/microseconds"));

  /// create publisher
  RCCHECK(rclc_publisher_init_default(
    &power_batteries_voltage_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/power/batteries/voltage"));

  RCCHECK(rclc_publisher_init_default(
    &power_thrusters_current_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/power/thrusters/current"));
  
  RCCHECK(rclc_publisher_init_default(
    &power_board_temperature_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/power/board/temperature"));

  RCCHECK(rclc_publisher_init_default(
    &power_teensy_temperature_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/power/teensy/temperature"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 100, &allocator)); // number arbitrarily set, idk what is the correct on yet, trial and error later on
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &propulsion_microseconds_subscriber, &propulsion_microseconds_msg, &propulsion_microseconds_callback, ON_NEW_DATA));


  return true;
}

void disconnectUSB() {
  USB1_USBCMD = 0;
}
void connectUSB() {
  USB1_USBCMD = 1;
}

void destroy_entities()
{
  disconnectUSB();
  delay(25);
  digitalWrite(9, HIGH);

  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&power_batteries_voltage_publisher, &node);
  rcl_publisher_fini(&power_thrusters_current_publisher, &node);
  rcl_publisher_fini(&power_board_temperature_publisher, &node);
  rcl_publisher_fini(&power_teensy_temperature_publisher, &node);
  rcl_subscription_fini(&propulsion_microseconds_subscriber, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);

  delay(25);
  digitalWrite(9, LOW);
  connectUSB();
}

void power_setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);

  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  // allocates correct message sizes and initialzies to 0, required or crashes

  propulsion_microseconds_msg.data.size = 8;
  propulsion_microseconds_msg.data.capacity = 8;
  propulsion_microseconds_msg.data.data = (int16_t*)malloc(propulsion_microseconds_msg.data.capacity * sizeof(int16_t));
  for (int i = 0; i < 8; i++) {
    propulsion_microseconds_msg.data.data[i] = 0;
  }

  power_batteries_voltage_msg.data.size = 2;
  power_batteries_voltage_msg.data.capacity = 2;
  power_batteries_voltage_msg.data.data = (float*)malloc(power_batteries_voltage_msg.data.capacity * sizeof(float));
  for (int i = 0; i < 2; i++) {
    power_batteries_voltage_msg.data.data[i] = 0.0;
  }

  power_thrusters_current_msg.data.size = 8;
  power_thrusters_current_msg.data.capacity = 8;
  power_thrusters_current_msg.data.data = (float*)malloc(power_thrusters_current_msg.data.capacity * sizeof(float));
  for (int i = 0; i < 8; i++) {
    power_thrusters_current_msg.data.data[i] = 0.0;
  }

  power_board_temperature_msg.data = 0.0;

  power_teensy_temperature_msg.data = 0.0;

  //allocates thrusters to 1500 in case of reset and allocates -2.0 to sensing to go under the -1.0 of unintiailized from drivers

  for (int i = 0; i < 8; i++) {
    propulsion_microseconds_msg.data.data[i] = 1500;
  }
  for (int i = 0; i < 2; i++) {
    power_batteries_voltage_msg.data.data[i] = -2.0;
  }
  for (int i = 0; i < 8; i++) {
    power_thrusters_current_msg.data.data[i] = -2.0;
  }
  power_board_temperature_msg.data = -2.0;
  power_teensy_temperature_msg.data = -2.0;

  state = WAITING_AGENT;
}

void power_loop() {
  switch (state) {
    case WAITING_AGENT:
      digitalWrite(2, HIGH);
      digitalWrite(3, LOW);
      digitalWrite(4, LOW);
      digitalWrite(5, LOW);
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      digitalWrite(8, HIGH);
      delay(200);
      digitalWrite(8, LOW);
      break;
    case AGENT_AVAILABLE:
      digitalWrite(2, LOW);
      digitalWrite(3, HIGH);
      digitalWrite(4, LOW);
      digitalWrite(5, LOW);
      delay(200);
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      digitalWrite(2, LOW);
      digitalWrite(3, LOW);
      digitalWrite(4, HIGH);
      digitalWrite(5, LOW);
      delay(200);
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      digitalWrite(2, LOW);
      digitalWrite(3, LOW);
      digitalWrite(4, LOW);
      digitalWrite(5, HIGH);
      delay(500);
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      digitalWrite(6, HIGH);
      delay(200);
      break;
  }

  if (state == AGENT_CONNECTED) {
    digitalWrite(LED_PIN, 1);
  } else {
    digitalWrite(LED_PIN, 0);
  }
}

#endif

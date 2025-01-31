#ifdef POWER_H

#include "power_main.h"

#include <Arduino.h>

#include "ThrusterControl.h"
#include "adc_sensors.h"
#include "TMP36.h"

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32_multi_array.h>

#define LED_PIN 13


#define ENABLE_VOLTAGE_SENSE true
#define ENABLE_CURRENT_SENSE true

ADCSensors adcSensors;
TMP36 temperatureSensor(23, 3.3);

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
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(100);
        error++;
    }
    digitalWrite(LED_PIN, HIGH);
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

  for (int i = 0; i < 8; i++) {
    microseconds[i] = msg->data.data[i];
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&power_batteries_voltage_publisher, &power_batteries_voltage_msg, NULL));
    RCSOFTCHECK(rcl_publish(&power_thrusters_current_publisher, &power_thrusters_current_msg, NULL));
    RCSOFTCHECK(rcl_publish(&power_board_temperature_publisher, &power_board_temperature_msg, NULL));
    RCSOFTCHECK(rcl_publish(&power_teensy_temperature_publisher, &power_teensy_temperature_msg, NULL));
  }
}

bool create_entities() {
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

void destroy_entities() {
  disconnectUSB();
  delay(25);
  updateThrusters(offCommand);

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
  connectUSB();
}

void senseData() {
  power_board_temperature_msg.data = temperatureSensor.readTemperature();

  power_teensy_temperature_msg.data = tempmonGetTemp();

  float* current_data = adcSensors.senseCurrent();
  for (size_t i = 0; i < 8; i++) {
      power_thrusters_current_msg.data.data[i] = current_data[i];
  }

  float* voltage_data = adcSensors.senseVoltage();
  for (size_t i = 0; i < 2; i++) {
      power_batteries_voltage_msg.data.data[i] = voltage_data[i];
  }
}

void power_setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  initThrusters();

  adcSensors.begin(ENABLE_VOLTAGE_SENSE, ENABLE_CURRENT_SENSE, &Wire1);
  temperatureSensor.begin();

  // Configure serial transport
  Serial.begin(115200);
  set_microros_transports();
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

  // first state
  state = WAITING_AGENT;
}

void power_loop() {
  senseData();
  updateThrusters(microseconds);

  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  if (state == AGENT_CONNECTED) {
    digitalWrite(LED_PIN, 1);
  } else {
    digitalWrite(LED_PIN, 0);
  }
}

#endif

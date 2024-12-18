#include "micro_ros_wrapper.h"

#include <Arduino.h>

// MicroROSWrapper Implementation
MicroROSWrapper::MicroROSWrapper() {
    set_microros_transports();
    current_state = WAITING_AGENT;
}

rcl_publisher_t* MicroROSWrapper::createPublisher(const char* topic_name, const rosidl_message_type_support_t* type_support) {
    RCCHECK(rclc_publisher_init_default(&publishers_[publisher_count], &node_, type_support, topic_name));
    publisher_count++;
    return &publishers_[publisher_count - 1];  // Return a pointer to the publisher
}


void MicroROSWrapper::publishData(const void* msg, rcl_publisher_t &publisher) {
    RCSOFTCHECK(rcl_publish(&publisher, msg, NULL));
}

rcl_subscription_t* MicroROSWrapper::createSubscriber(const char* topic_name, const rosidl_message_type_support_t* type_support, void (*callback)(const void*)) {
    RCCHECK(rclc_subscription_init_default(&subscriptions_[subscription_count], &node_, type_support, topic_name));
    RCCHECK(rclc_executor_add_subscription(&executor_, &subscriptions_[subscription_count], nullptr, callback, ON_NEW_DATA));
    subscription_count++;
    return &subscriptions_[subscription_count - 1];
}

void MicroROSWrapper::spin() {
    RCCHECK(rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(100)));
}

void MicroROSWrapper::errorLoop() {
    int error = 0;

    while (error < 20) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(100);

        error++;
    }

    digitalWrite(LED_BUILTIN, HIGH);
}

bool MicroROSWrapper::createEntities(char* node_name) {
    allocator_ = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support_, 0, NULL, &allocator_));
    RCCHECK(rclc_node_init_default(&node_, node_name, "", &support_));
    executor_ = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor_, &support_.context, 1, &allocator_));

    node_name_ = node_name;

    return true;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"

void MicroROSWrapper::destroyEntities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support_.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  for (int i = 0; i < publisher_count; i++) {
    rcl_publisher_fini(&publishers_[i], &node_);
  }

  publisher_count = 0;

  for (int i = 0; i < subscription_count; i++) {
    rcl_subscription_fini(&subscriptions_[i], &node_);
  }

  subscription_count = 0;

  rclc_executor_fini(&executor_);
  rcl_node_fini(&node_);
  rclc_support_fini(&support_);
}

#pragma GCC diagnostic pop

void MicroROSWrapper::handleStateMachine() {
    switch (state) {
    case WAITING_AGENT:
      state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
      delay(200);
      break;
    case AGENT_AVAILABLE:
      state = (true == createEntities(node_name_)) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == AGENT_CONNECTED) {
        requestsOpen = true;
      }
      if (state == WAITING_AGENT) {
        destroyEntities();
      };
      break;
    case AGENT_CONNECTED:
      state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      delay(200);
      if (state == AGENT_CONNECTED) {
        executionRequest = true;
        rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroyEntities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }
}

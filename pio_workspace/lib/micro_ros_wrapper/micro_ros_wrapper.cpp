#include "micro_ros_wrapper.h"

MicroROSWrapper::MicroROSWrapper() {
    set_microros_transports();
}

void MicroROSWrapper::init(const char* node_name) {
    allocator_ = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support_, 0, NULL, &allocator_));
    RCCHECK(rclc_node_init_default(&node_, node_name, "", &support_));
    RCCHECK(rclc_executor_init(&executor_, &support_.context, MAX_HANDLE_COUNT, &allocator_));
}

void MicroROSWrapper::spin() {
    RCCHECK(rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(10)));
}

rcl_publisher_t* MicroROSWrapper::createPublisher(const char* topic_name, const rosidl_message_type_support_t* type_support) {
    if (publisher_count_ == MAX_PUBLISHER_COUNT) {
        return nullptr;
    }
    RCCHECK(rclc_publisher_init_default(&publishers_[publisher_count_], &node_, type_support, topic_name));
    publisher_count_++;
    return &publishers_[publisher_count_ - 1];
}

rcl_subscription_t* MicroROSWrapper::createSubscriber(const char* topic_name, const rosidl_message_type_support_t* type_support, void (*callback)(const void*)) {
    if (subscription_count_ == MAX_SUBSCRIPTION_COUNT) {
        return nullptr;
    }
    RCCHECK(rclc_subscription_init_default(&subscriptions_[subscription_count_], &node_, type_support, topic_name));
    RCCHECK(rclc_executor_add_subscription(&executor_, &subscriptions_[subscription_count_], nullptr, callback, ON_NEW_DATA));
    subscription_count_++;
    return &subscriptions_[subscription_count_ - 1];
}

void MicroROSWrapper::publishData(const void* msg, rcl_publisher_t &publisher) {
    RCSOFTCHECK(rcl_publish(&publisher, msg, NULL));
}

bool MicroROSWrapper::pingAgent() {
    return RMW_RET_OK == rmw_uros_ping_agent(100, 1);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"
void MicroROSWrapper::destroyMicroROS() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support_.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  for (int i = 0; i < publisher_count_; i++) {
    rcl_publisher_fini(&publishers_[i], &node_);
  }

  publisher_count_ = 0;

  for (int i = 0; i < subscription_count_; i++) {
    rcl_subscription_fini(&subscriptions_[i], &node_);
  }

  subscription_count_ = 0;

  rclc_executor_fini(&executor_);
  rcl_node_fini(&node_);
  rclc_support_fini(&support_);
}
#pragma GCC diagnostic pop

void MicroROSWrapper::errorLoop_() {
    int error = 0;
    while (error < 10) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(100);

        error++;
    }
    digitalWrite(LED_BUILTIN, HIGH);
}

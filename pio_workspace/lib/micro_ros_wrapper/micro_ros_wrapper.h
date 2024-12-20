#ifndef MICRO_ROS_WRAPPER_H
#define MICRO_ROS_WRAPPER_H

#include <micro_ros_arduino.h>

#include <Arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#define MAX_HANDLE_COUNT 101
#define MAX_PUBLISHER_COUNT 100
#define MAX_SUBSCRIPTION_COUNT 100

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { errorLoop_(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {} }

class MicroROSWrapper {
public:
    MicroROSWrapper();
    void init(const char* node_name);
    void spin();

    rcl_publisher_t* createPublisher(const char* topic_name, const rosidl_message_type_support_t* type_support);
    rcl_subscription_t* createSubscriber(const char* topic_name, const rosidl_message_type_support_t* type_support, void (*callback)(const void*));
    void publishData(const void* msg, rcl_publisher_t &publisher);

    bool pingAgent();
    void destroyMicroROS();

private:
    rclc_executor_t executor_;
    rclc_support_t support_;
    rcl_allocator_t allocator_;
    rcl_node_t node_;

    rcl_publisher_t publishers_[MAX_PUBLISHER_COUNT];
    rcl_subscription_t subscriptions_[MAX_SUBSCRIPTION_COUNT];

    int publisher_count_ = 0;
    int subscription_count_ = 0;

    void errorLoop_();
};

#endif

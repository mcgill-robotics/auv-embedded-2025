#ifndef MICRO_ROS_WRAPPER_H
#define MICRO_ROS_WRAPPER_H

#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { errorLoop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {} }

class MicroROSWrapper {
public:
    MicroROSWrapper(const char* node_name);
    void spin();
    rcl_publisher_t* createPublisher(const char* topic_name, const rosidl_message_type_support_t* type_support);
    rcl_subscription_t* createSubscriber(const char* topic_name, const rosidl_message_type_support_t* type_support, void (*callback)(const void*));

    void publishData(const void* msg, rcl_publisher_t &publisher);

private:
    rclc_executor_t executor_;
    rclc_support_t support_;
    rcl_allocator_t allocator_;
    rcl_node_t node_;

    rcl_publisher_t publishers_[50];
    rcl_subscription_t subscriptions_[50];

    int publisher_count = 0;
    int subscription_count = 0;

    void errorLoop();
};

#endif

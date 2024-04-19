//base for acuator board
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

bool close_flag = false;
float current_data = 0.0;
ros::NodeHandle nh;

void messageCb(const std_msgs::Bool & close){
  close_flag = close.data;
}
std_msgs::Float32 current_msg;
ros::Publisher sendData("current_data", &current_msg);
ros::Subscriber<std_msgs::Bool> sub_close("close_sub", &messageCb);


 void setup() {
//-------------------------------------------
  nh.initNode();
  nh.subscribe(sub_close);
  nh.advertise(sendData);
}

void loop() {
  if(close_flag){
    current_data = 1;
    current_msg.data = current_data;
  }
  sendData.publish(&current_msg);
  nh.spinOnce();
  delay(500);
}

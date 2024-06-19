#include <Arduino.h>

#include "Config_ROS.h"

ros::NodeHandle nh;
std_msgs::Int32 msg;
const uint16_t serverPort = 11411;

void messageCb(const std_msgs::Int32& data, Servo& servo) {
  int error_dist = data.data;
  servo.write(servo.read() + 5 * static_cast<int>(tanh(0.025 * error_dist)));
}

void horRotCb(const std_msgs::Int32& data) {
  messageCb(data, horizontal_servo);
}

void verRotCb(const std_msgs::Int32& data) {
  messageCb(data, vertical_servo);
}

void setupROS() {
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.subscribe(sub_hor_rot);
  nh.subscribe(sub_ver_rot);
}

ros::Subscriber<std_msgs::Int32> sub_hor_rot("/SPS/hor_rot", &horRotCb);
ros::Subscriber<std_msgs::Int32> sub_ver_rot("/SPS/ver_rot", &verRotCb);

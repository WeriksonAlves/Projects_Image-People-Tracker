#include "Config_ROS.h"

ros::NodeHandle nh;

std_msgs::Int32 msg;

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

ros::Subscriber<std_msgs::Int32> sub_hor_rot("/SPS/hor_rot", &horRotCb);
ros::Subscriber<std_msgs::Int32> sub_ver_rot("/SPS/ver_rot", &verRotCb);

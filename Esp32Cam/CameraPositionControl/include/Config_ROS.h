#ifndef CONFIG_ROS_H
#define CONFIG_ROS_H

#include <ros.h>
#include <std_msgs/Int32.h>
#include <Servo.h>
#include "Config_Servo.h"

extern ros::NodeHandle nh;

extern std_msgs::Int32 msg;

// Callback function for horizontal rotation
void horRotCb(const std_msgs::Int32& data);

// Callback function for vertical rotation
void verRotCb(const std_msgs::Int32& data);

// Subscriber for horizontal rotation topic
extern ros::Subscriber<std_msgs::Int32> sub_hor_rot;

// Subscriber for vertical rotation topic
extern ros::Subscriber<std_msgs::Int32> sub_ver_rot;

#endif // CONFIG_ROS_H

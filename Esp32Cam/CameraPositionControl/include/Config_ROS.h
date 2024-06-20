#include <Arduino.h>

#ifndef CONFIG_ROS_H
#define CONFIG_ROS_H

#include <ros.h>
#include <std_msgs/Int32.h>
#include <ESP32Servo.h>
#include "Config_Servo.h"
#include "Config_WiFi.h"

extern const uint16_t serverPort; // Declaration

extern ros::NodeHandle nh;
extern std_msgs::Int32 msg;

void setupROS();
void horRotCb(const std_msgs::Int32& data);
void verRotCb(const std_msgs::Int32& data);

extern ros::Subscriber<std_msgs::Int32> sub_hor_rot;
extern ros::Subscriber<std_msgs::Int32> sub_ver_rot;

#endif // CONFIG_ROS_H

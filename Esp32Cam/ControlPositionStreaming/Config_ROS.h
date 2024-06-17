// ===========================
// ROS settings
// ===========================

const uint16_t serverPort = 11411; // CONEXAO TCP

ros::NodeHandle nh;
//std_msgs::String msg;

void horRotCb(const std_msgs::String& data) {
//  messageCb(data, servo_h);
}

void verRotCb(const std_msgs::String& data) {
//  messageCb(data, servo_v);
}

ros::Subscriber<std_msgs::String> sub_hor_rot("/SPS/hor_rot", &horRotCb);
ros::Subscriber<std_msgs::String> sub_ver_rot("/SPS/ver_rot", &verRotCb);

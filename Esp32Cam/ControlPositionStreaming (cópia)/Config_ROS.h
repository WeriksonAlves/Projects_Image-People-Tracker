// ===========================
// ROS settings
// ===========================

#include <math.h>

const uint16_t serverPort = 11411; // CONEXAO TCP

ros::NodeHandle nh;
std_msgs::Int32 msg;


void messageCb(const std_msgs::Int32& data, Servo& servo) {
  int error_dist = data.data;
  servo.write(servo.read() + 5*(int)tanh(0.025*error_dist));
}

void horRotCb(const std_msgs::Int32& data) {
  messageCb(data, servo_h);
}

void verRotCb(const std_msgs::Int32& data) {
  messageCb(data, servo_v);
}

ros::Subscriber<std_msgs::Int32> sub_hor_rot("/SPS/hor_rot", &horRotCb);
ros::Subscriber<std_msgs::Int32> sub_ver_rot("/SPS/ver_rot", &verRotCb);


//// ===========================
//// ROS settings
//// ===========================
//#include <math.h>
//#include <std_msgs/Int32.h>
//
//const uint16_t serverPort = 11411; // CONEXAO TCP
//
//ros::NodeHandle nh;
//std_msgs::Int32 msg;
//
//ros::Publisher pub_hor_angle("/SPS/hor_angle", &msg);
//ros::Publisher pub_ver_angle("/SPS/ver_angle", &msg);
//
//void publishServoAngles() {
//    // Publish horizontal servo angle
//    msg.data = servo_h.read();
//    pub_hor_angle.publish(&msg);
//
//    // Publish vertical servo angle
//    msg.data = servo_v.read();
//    pub_ver_angle.publish(&msg);
//}
//
//void messageCb(const std_msgs::Int32& data, Servo& servo) {
//    int error_dist = data.data;
//    servo.write(servo.read() + 5 * (int)tanh(0.025 * error_dist));
//}
//
//void horRotCb(const std_msgs::Int32& data) {
//    messageCb(data, servo_h);
//}
//
//void verRotCb(const std_msgs::Int32& data) {
//    messageCb(data, servo_v);
//}
//
//ros::Subscriber<std_msgs::Int32> sub_hor_rot("/SPS/hor_rot", &horRotCb);
//ros::Subscriber<std_msgs::Int32> sub_ver_rot("/SPS/ver_rot", &verRotCb);

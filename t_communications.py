#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from factory_context import factory_context
from std_msgs.msg import Int16MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler

rospy.init_node('Planner', anonymous=True)
pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)
velocity = Twist()
planner = factory_context.run(11, 11, 4, 3, [(5, 1), (4, 5), (1, 7), (7, 9)])


pose = np.zeros(3, dtype=np.float32) # x, y, yaw
Rot = np.eye(2)
displaced_point = 10/100

def talker():
    path : np.ndarray = np.array(planner.get_path()) / 2 + .25
    path : list = path.tolist()[::-1]

    #path = list([[0, 0], [5.20, 0], [5.0, -2], [5.20, 0]])

    vel_d = .12
    Xd = np.array(path.pop())


    rate = rospy.Rate(10) # 10hz
    while len(path) >= 0 and not rospy.is_shutdown():
        if np.linalg.norm(Xd - pose[:2]) < 5/100:
            if len(path) == 0:
                break
            Xd = path.pop()
            Xd = np.array(Xd)
            print(Xd)


        Xtil = Xd - pose[:2]
        Xtil_normalized = Xtil / np.linalg.norm(Xtil)
        desired_velocity = vel_d * Xtil_normalized

        U = np.dot(np.linalg.inv(Rot), desired_velocity)

        velocity.linear.x =  U[0]
        velocity.angular.z = U[1]

        pub.publish(velocity)
        rate.sleep()



def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/RosAria/pose", Odometry, callback)


def update_pose(msg : Odometry, pose):
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (_, _, yaw) = euler_from_quaternion(orientation_list)
    pose[2] = yaw

def update_rotation():
    yaw = pose[2]
    Rot[0, 0] = np.cos(yaw)
    Rot[0, 1] = -np.sin(yaw) * displaced_point
    Rot[1, 0] = np.sin(yaw)
    Rot[1, 1] = np.cos(yaw) * displaced_point

def callback(data : Odometry):
    pose[0] = data.pose.pose.position.x
    pose[1] = data.pose.pose.position.y
    update_pose(data, pose)
    update_rotation()


if __name__ == '__main__':
    try:
        listener()
        talker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
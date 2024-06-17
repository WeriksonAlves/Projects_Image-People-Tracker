import rospy
from std_msgs.msg import Int32

class CommunicationEspCam:
    def __init__(self, horizontal_pub: rospy.Publisher, vertical_pub: rospy.Publisher) -> None:
        self.horizontal_pub = horizontal_pub
        self.vertical_pub = vertical_pub

    def perform_action(self, action: str) -> None:
        """
        Moves the servo based on the specified action.

        Args:
            action (str): The action to move the servo. 
                '0': Person is centered.
                '+1': Turn the horizontal servo counterclockwise.
                '-1': Turn the horizontal servo clockwise.
                '+2': Turn the vertical servo clockwise.
                '-2': Turn the vertical servo counterclockwise.

        Returns:
            None
        """
        action_messages = {
            '0': "No servo should rotate.",
            '+1': "Turn the horizontal servo counterclockwise.",
            '-1': "Turn the horizontal servo clockwise.",
            '+2': "Turn the vertical servo clockwise.",
            '-2': "Turn the vertical servo counterclockwise.",
        }

        # print(action_messages.get(action, "Invalid action."))

        # Send the command to the servo using the ROS communication module.
        if action == '0':
            self.horizontal_pub.publish(action)
            self.vertical_pub.publish(action)
        elif action in ['+1', '-1']:
            self.horizontal_pub.publish(action)
        elif action in ['+2', '-2']:
            self.vertical_pub.publish(action)
        else:
            print("Invalid action.")

    def perform_action2(self, num_servo: int, error_dist: int):
        
        if num_servo == 1:
            l = Int32()
            l.data = error_dist
            self.horizontal_pub.publish(l)
        else:
            l1 = Int32()
            l2 = Int32()
            l1.data = error_dist[0]
            l2.data = error_dist[1]
            self.horizontal_pub.publish(l1)
            self.vertical_pub.publish(l2)


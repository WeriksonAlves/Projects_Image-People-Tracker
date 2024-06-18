import rospy
from std_msgs.msg import Int32
from typing import Union, Tuple


class CommunicationEspCam:
    def __init__(self, horizontal_publisher: rospy.Publisher, vertical_publisher: rospy.Publisher) -> None:
        """
        Initializes the CommunicationEspCam object.

        Args:
            horizontal_publisher (rospy.Publisher): ROS publisher for horizontal servo control.
            vertical_publisher (rospy.Publisher): ROS publisher for vertical servo control.
        """
        self.horizontal_publisher = horizontal_publisher
        self.vertical_publisher = vertical_publisher

    def perform_action(self, action: str, distance_to_center: int, show_message: bool = False) -> None:
        """
        Sends control commands to the servos based on the specified action.

        Args:
            action (str): Action command to control the servos.
            distance_to_center (int): Distance to center for the servo adjustments.
            show_message (bool): Flag to indicate whether to print the action message. Defaults to False.

        Returns:
            None
        """
        # Define action messages
        action_messages = {
            '+1': "Turn the horizontal servo clockwise.",
            '-1': "Turn the horizontal servo counterclockwise.",
            '+2': "Turn the vertical servo counterclockwise.",
            '-2': "Turn the vertical servo clockwise.",
        }

        # Print the action message if show_message is True
        if show_message:
            print(action_messages.get(action, "Invalid action."))

        # Publish the control commands to the servos
        if action in ['+1', '-1']: # Horizontal servo control
            horizontal_message = Int32(data=distance_to_center)
            self.horizontal_publisher.publish(horizontal_message)
        elif action in ['+2', '-2']: # Vertical servo control
            vertical_message = Int32(data=distance_to_center)
            self.vertical_publisher.publish(vertical_message)
        else: # Invalid action
            print("Invalid action.")

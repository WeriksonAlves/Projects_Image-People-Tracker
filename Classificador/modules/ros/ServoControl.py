import rospy
import numpy as np
from std_msgs.msg import Int32
from typing import Union, Tuple


class CommunicationEspCam:
    def __init__(self, horizontal_publisher: rospy.Publisher, vertical_publisher: rospy.Publisher, dir_rot: int = 1) -> None:
        """
        Initializes the CommunicationEspCam object.

        Args:
            horizontal_publisher (rospy.Publisher): ROS publisher for horizontal servo control.
            vertical_publisher (rospy.Publisher): ROS publisher for vertical servo control.
        """
        self.horizontal_publisher = horizontal_publisher
        self.vertical_publisher = vertical_publisher
        self.dir_rot = dir_rot

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

        gains = [4, 1/30]
        signal = round(self.dir_rot * gains[0] * np.tanh(gains[1] * distance_to_center))

        # Print the action message if show_message is True
        if show_message:
            print(action_messages.get(action, "Invalid action."), distance_to_center, signal)

        # Publish the control commands to the servos
        if action in ['+1', '-1']: # Horizontal servo control
            horizontal_message = Int32(data=signal)
            self.horizontal_publisher.publish(horizontal_message)
        elif action in ['+2', '-2']: # Vertical servo control
            vertical_message = Int32(data=signal)
            self.vertical_publisher.publish(vertical_message)
        else: # Invalid action
            print("Invalid action.")

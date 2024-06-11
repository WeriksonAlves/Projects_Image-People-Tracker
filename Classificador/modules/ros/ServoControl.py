import rospy


class CommunicationEspCam:
    def __init__(self, pub_hor_rot: rospy.Publisher, pub_ver_rot: rospy.Publisher) -> None:
        self.pub_hor_rot = pub_hor_rot
        self.pub_ver_rot = pub_ver_rot

    def action(self, action: str) -> None:
        """
        Moves the servo in the specified action.

        Args:
            action (int): The action to move the servo. 
                0: Person is centered.
                +1: Turn the horizontal servo counterclockwise.
                -1: Turn the horizontal servo clockwise.
                +2: Turn the vertical servo clockwise.
                -2: Turn the vertical servo counterclockwise.

        Returns:
            None

        """
        action_messages = {
            '0': "Person is centered.",
            '+1': "Turn the horizontal servo counterclockwise.",
            '-1': "Turn the horizontal servo clockwise.",
            '+2': "Turn the vertical servo clockwise.",
            '-2': "Turn the vertical servo counterclockwise.",
        }

        print(action_messages.get(action, "Invalid direction."))
        
        # Send the command to the servo using the ROS communication module.
        if action == 0:
            self.pub_hor_rot.publish(action)
            self.pub_ver_rot.publish(action)
        elif action[1] == '1':
            self.pub_hor_rot.publish(action)
        elif action[1] == '2':
            self.pub_ver_rot.publish(action)
        else:
            print("Invalid direction.")
            

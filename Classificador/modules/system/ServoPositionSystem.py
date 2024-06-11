import os
from typing import Tuple, Union
import numpy as np

class ServoPositionSystem:
    def __init__(self, num_servos: int = 0):
        self.num_servos = num_servos
        self.enable = self.num_servos != 0
        self.centered = False
        self.distance_to_center = (0, 0)
    
    def is_person_centered(self, captured_frame: np.ndarray, bounding_box: Tuple[int, int, int, int]) -> Union[None, Tuple[bool, Tuple[int, int]]]:
        """
        Check if a person is centered within a given frame.
        
        Args:
            captured_frame (np.ndarray): The captured frame as a NumPy array.
            bounding_box (tuple): The bounding box coordinates of the person in the frame.
        
        Returns:
            bool: True if the person is centered, False otherwise.
            tuple: The distance of the person from the center of the frame.
        """
        if not self.enable:
            return

        frame_height, frame_width, _ = captured_frame.shape
        box_x, box_y, box_w, box_h = bounding_box
        
        frame_center = (frame_width // 2, frame_height // 2)
        
        if self.num_servos == 1:
            self.distance_to_center = (box_x - frame_center[0],)
            reference_distance = np.linalg.norm([box_w // 2])
        else:
            self.distance_to_center = (box_x - frame_center[0], box_y - frame_center[1])
            reference_distance = np.linalg.norm([box_w // 2, box_h // 2])

        distance = np.linalg.norm(self.distance_to_center)
        
        self.centered = distance < reference_distance
        print("Person is centered." if self.centered else "Person is NOT centered.")
        
        self.control_servo()
    
    def control_servo(self) -> None:
        """
        Controls the servo based on the current state of the system.

        - If the system is not activated, the method will return immediately.
        - If the system is centered, the servo remains stationary.
        - If the system is not centered, the servo is moved horizontally based on the 
        direction of rotation to correct the centering.
        - If there are 2 servos, the servo will also be moved vertically based on the 
        direction of rotation to correct the centering.
        
        Returns:
            None
        """
        if not self.enable:
            return

        if self.centered:
            self.move_servo(0)
        else:
            horizontal_direction = 1 if self.distance_to_center[0] < 0 else -1
            self.move_servo(horizontal_direction)

            if self.num_servos > 1:
                vertical_direction = -2 if self.distance_to_center[1] < 0 else 2
                self.move_servo(vertical_direction)
    
    def move_servo(self, direction: int) -> None:
        """
        Moves the servo in the specified direction.

        Args:
            direction (int): The direction to move the servo. 
                0: Person is centered.
                1: Turn the horizontal servo counterclockwise.
                -1: Turn the horizontal servo clockwise.
                2: Turn the vertical servo clockwise.
                -2: Turn the vertical servo counterclockwise.

        Returns:
            None

        """
        direction_messages = {
            0: "Person is centered.",
            1: "Turn the horizontal servo counterclockwise.",
            -1: "Turn the horizontal servo clockwise.",
            2: "Turn the vertical servo clockwise.",
            -2: "Turn the vertical servo counterclockwise.",
        }
        print(direction_messages.get(direction, "Invalid direction."))

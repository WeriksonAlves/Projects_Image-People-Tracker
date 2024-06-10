import os
from typing import Union
import numpy as np

class ServoPositionSystem:
    def __init__(self, frame_original: np.ndarray, frame_person: np.ndarray, servo_enable: bool = False, servo_number: int = 1):
        self.frame_original = frame_original
        self.frame_person = frame_person
        self.servo_enable = servo_enable
        self.servo_number = servo_number
    
    # Checks if the image of the person is centered in relation to the original image
    def is_person_centered(self, frame_original: np.ndarray, frame_person: np.ndarray) -> bool:
        """
        Check if the person is centered in relation to the original image.

        Args:
            frame_original (np.ndarray): The original frame.
            frame_person (np.ndarray): The person frame.

        Returns:
            bool: Whether the person is centered.
        """
        height, width, _ = frame_original.shape
        person_height, person_width, _ = frame_person.shape
        
        center_original = (int(width / 2), int(height / 2))
        center_person = (int(person_width / 2), int(person_height / 2))

        current_position = (center_original[0] - center_person[0], center_original[1] - center_person[1])
        distance = np.linalg.norm(current_position)
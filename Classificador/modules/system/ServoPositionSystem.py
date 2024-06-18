from typing import Tuple, Union
import numpy as np
from ..ros.ServoControl import CommunicationEspCam


class ServoPositionSystem:
    def __init__(self, num_servos: int = 0, com_esp_cam: CommunicationEspCam = None):
        """
        Initializes a ServoPositionSystem object.

        Args:
            num_servos (int): The number of servos in the system. Defaults to 0.
            com_esp_cam (CommunicationEspCam): An instance of CommunicationEspCam class for communication with ESP-CAM. Defaults to None.
        """
        self.num_servos = num_servos
        self.com_esp_cam = com_esp_cam
        self.enabled = self.num_servos != 0

    def check_person_centered(self, captured_frame: np.ndarray, bounding_box: Tuple[int, int, int, int]) -> None:
        """
        Checks if a person is centered in the captured frame and adjusts the servo position accordingly.

        Args:
            captured_frame (np.ndarray): The captured frame as a numpy array.
            bounding_box (Tuple[int, int, int, int]): The bounding box coordinates of the detected person (x, y, width, height).

        Returns:
            None
        """
        # If the system is not enabled, return
        if not self.enabled:
            return
        
        # Calculate the center of the frame
        frame_height, frame_width, _ = captured_frame.shape
        frame_center = (frame_width // 2, frame_height // 2)
        
        # Obtain the bounding box coordinates
        box_x, box_y, box_width, box_height = bounding_box

        # Calculate the distance horizontal to the center and move the servo accordingly
        distance_to_center_h = box_x - frame_center[0]
        horizontal_direction = '-1' if distance_to_center_h < 0 else '+1'
        self.com_esp_cam.perform_action(horizontal_direction, distance_to_center_h, True)

        # Calculate the distance vertical to the center and move the servo accordingly
        if self.num_servos > 1:
            distance_to_center_v = box_y - frame_center[1]
            vertical_direction = '+2' if distance_to_center_v < 0 else '-2'
            self.com_esp_cam.perform_action(vertical_direction, distance_to_center_v)
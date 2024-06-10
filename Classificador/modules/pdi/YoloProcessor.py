from .interfaces import InterfaceTrack
from ultralytics import YOLO
from collections import defaultdict

import numpy as np
import cv2

class YoloProcessor(InterfaceTrack):
    """
    YOLO processor class for tracking.
    """
    def __init__(self, file_YOLO: str):
        """
        Initialize YOLO processor.

        Args:
            yolo_model (YOLO): The YOLO model.
        """
        self.yolo_model = YOLO(file_YOLO)
        self.track_history = defaultdict(list)
    
    def find_people(self, captured_frame: np.ndarray, persist: bool = True, verbose: bool = False) -> list:
        """
        Find people in captured frame.

        Args:
            captured_frame (np.ndarray): The captured frame.
            persist (bool): Whether to persist the results.
            verbose (bool): Whether to output verbose information.

        Returns:
            List: A list of people found in the frame.
        """
        return self.yolo_model.track(captured_frame, persist=persist, verbose=verbose)
    
    def identify_operator(self, results_people: list) -> np.ndarray:
        """
        Identify operator.

        Args:
            results_people: Results of people found in the frame.
            
        Returns:
            np.ndarray: Image results.
        """
        return results_people[0].plot()
    
    def track_operator(self, results_people: list, results_identifies: np.ndarray, captured_frame: np.ndarray, length: int = 90) -> np.ndarray:
        """
        Tracks the operator identified in the captured frame.

        Args:
            results_people (list): List of detected people results.
            results_identifies (np.ndarray): Array of identification results.
            captured_frame (np.ndarray): The captured frame.
            length (int, optional): Length of the track history. Defaults to 90.

        Returns:
            np.ndarray: The flipped person ROI and the coordinates of the bounding box (x, y, w, h).
        """
        boxes = results_people[0].boxes.xywh.cpu()
        track_ids = results_people[0].boxes.id.int().cpu().tolist()
        for box, track_id in zip(boxes, track_ids):
            x, y, w, h = map(int, box)
            track = self.track_history[track_id]
            track.append((x + w // 2, y + h // 2))
            track = track[-length:]
            points = np.vstack(track).astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(results_identifies, [points], isClosed=False, color=(230, 230, 230), thickness=10)
            person_roi = captured_frame[(y - h // 2):(y + h // 2), (x - w // 2):(x + w // 2)]
            break
        return cv2.flip(person_roi, 1), (x, y, w, h)
    
    def is_person_centered(self, captured_frame: np.ndarray, box: tuple, number_servo: int = 1) -> bool:
        """
        Checks if a person is centered in the captured frame based on the given bounding box coordinates.

        Args:
            captured_frame (np.ndarray): The captured frame as a NumPy array.
            box (tuple): The bounding box coordinates of the person in the format (x, y, w, h).
            number_servo (int, optional): The number of the servo. Defaults to 1.

        Returns:
            bool: True if the person is not centered, False otherwise.
            tuple: The distance of the person from the center of the frame.

        """
        height, width, _ = captured_frame.shape
        x, y, w, h = box
        center_original = (int(width / 2), int(height / 2))
        if number_servo == 1:
            dist_center_person = (x - center_original[0])
            reference_dist = np.linalg.norm((w//2))
        else:
            dist_center_person = (x - center_original[0], y - center_original[1])
            reference_dist = np.linalg.norm((w//2, h//2))
        
        distance = np.linalg.norm(dist_center_person)
        if distance < reference_dist:
            print("Person is centered.")
            return False, dist_center_person
        else:
            print("Person is NOT centered.")
            return True, dist_center_person
        
        

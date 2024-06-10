from abc import ABC, abstractmethod


class InterfaceTrack(ABC):
    """
    Abstract base class for tracking processors.
    """
    @abstractmethod
    def find_people(self, captured_frame):
        """
        Abstract method to find people in captured frames.

        Args:
            captured_frame (np.ndarray): The captured frame.

        Returns:
            List: A list of people found in the frame.
        """
        pass
    
    @abstractmethod
    def identify_operator(self, results_people):
        """
        Abstract method to identify an operator.

        Args:
            results_people: Results of people found in the frame.
        """
        pass
    
    @abstractmethod
    def track_operator(self, results_people, results_identifies, captured_frame):
        """
        Abstract method to track operators in frames.

        Args:
            results_people: Results of people found in the frame.
            results_identifies (np.ndarray): Image results.
            captured_frame (np.ndarray): The captured frame.
        """
        pass
    
    @abstractmethod
    def is_person_centered(self, frame_original, frame_person):
        """
        Abstract method to check if the person is centered in relation to the original image.
        
        Args:
            frame_original (np.ndarray): The original frame.
            frame_person (np.ndarray): The person frame.
        
        Returns:
            bool: Whether the person is centered.
        """
        pass
    

class InterfaceFeature(ABC):
    """
    Abstract base class for feature extraction processors.
    """
    @abstractmethod
    def find_features(self, projected_window):
        """
        Abstract method to find features in a projected window.

        Args:
            projected_window (np.ndarray): The projected window.

        Returns:
            Tuple: A tuple containing the found features.
        """
        pass
    
    @abstractmethod
    def draw_features(self, projected_window, results):
        """
        Abstract method to draw features on a projected window.

        Args:
            projected_window (np.ndarray): The projected window.
            results: The results to draw.
            
        Returns:
            np.ndarray: The modified projected window.
        """
        pass
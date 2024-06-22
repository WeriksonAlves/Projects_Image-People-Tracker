import os
import threading
from typing import Union
import cv2  # Ensure cv2 is imported
import numpy as np
from .SystemSettings import *
from ..auxiliary.FileHandler import FileHandler
from ..auxiliary.TimeFunctions import TimeFunctions
from ..classifier.interfaces import InterfaceClassifier
from ..gesture.DataProcessor import DataProcessor
from ..gesture.GestureAnalyzer import GestureAnalyzer
from ..gesture.FeatureExtractor import FeatureExtractor
from ..pdi.interfaces import InterfaceTrack, InterfaceFeature
from ..system.ServoPositionSystem import ServoPositionSystem


class GestureRecognitionSystem:
    def __init__(self, config: InitializeConfig, operation: Union[ModeDataset, ModeValidate, ModeRealTime], 
                file_handler: FileHandler, current_folder: str, data_processor: DataProcessor, 
                time_functions: TimeFunctions, gesture_analyzer: GestureAnalyzer, tracking_processor: InterfaceTrack, 
                feature: InterfaceFeature, classifier: InterfaceClassifier = None, sps: ServoPositionSystem = None):
        
        self._initialize_camera(config)
        self._initialize_operation(operation)
        
        self.file_handler = file_handler
        self.current_folder = current_folder
        self.data_processor = data_processor
        self.time_functions = time_functions
        self.gesture_analyzer = gesture_analyzer
        self.tracking_processor = tracking_processor
        self.feature = feature
        self.classifier = classifier
        self.sps = sps
        
        self._initialize_simulation_variables()
        self._initialize_storage_variables()

        # For threading
        self.frame_lock = threading.Lock()
        self.frame_captured = None
        
        # Loop control flag
        self.loop = True
        
        # Thread for reading images
        self.image_thread = threading.Thread(target=self._read_image_thread)
        self.image_thread.daemon = True
        self.image_thread.start()
        
        # Thread for previewing images
        self.preview_thread = threading.Thread(target=self._image_preview_thread)
        self.preview_thread.daemon = True
        self.preview_thread.start()

    def _initialize_camera(self, config: InitializeConfig) -> None:
        self.cap = config.cap
        self.fps = config.fps
        self.dist = config.dist
        self.length = config.length

    def _initialize_operation(self, operation: Union[ModeDataset, ModeValidate, ModeRealTime]) -> None:
        self.mode = operation.mode
        if self.mode == 'D':
            self.database = operation.database
            self.file_name_build = operation.file_name_build
            self.max_num_gest = operation.max_num_gest
            self.dist = operation.dist
            self.length = operation.length
        elif self.mode == 'V':
            self.database = operation.database
            self.proportion = operation.proportion
            self.files_name = operation.files_name
            self.file_name_val = operation.file_name_val
        elif self.mode == 'RT':
            self.database = operation.database
            self.proportion = operation.proportion
            self.files_name = operation.files_name
        else:
            raise ValueError("Invalid mode")

    def _initialize_simulation_variables(self) -> None:
        self.stage = 0
        self.num_gest = 0
        self.dist_virtual_point = 1
        self.hands_results = None
        self.pose_results = None
        self.time_gesture = None
        self.time_action = None
        self.y_val = None
        self.frame_captured = None
        self.center_person = False
        self.y_predict = []
        self.time_classifier = []

    def _initialize_storage_variables(self) -> None:
        self.hand_history, _, self.wrists_history, self.sample = self.data_processor.initialize_data(self.dist, self.length)

    def run(self):
        if self.mode == 'D':
            self._initialize_database()
            self.loop = True
        elif self.mode == 'RT':
            self._load_and_fit_classifier()
            self.loop = True
            self.servo_enabled = True
        elif self.mode == 'V':
            self._validate_classifier()
            self.loop = False
            self.servo_enabled = False
        else:
            print(f"Operation mode invalid!")
            self.loop = False
            self.servo_enabled = False
            
        t_frame = self.time_functions.tic()
        while self.loop:
            if self.time_functions.toc(t_frame) > (1 / self.fps):
                print(f"FPS: {1 / self.time_functions.toc(t_frame):.3f} and Time per frame: {self.time_functions.toc(t_frame):.3f}")
                t_frame = self.time_functions.tic()
                
                if self.mode == "B":
                    if self.num_gest == self.max_num_gest:
                        break
                
                self._process_stage()

        self.cap.release()
        cv2.destroyAllWindows()

    def _initialize_database(self):
        self.target_names, self.y_val = self.file_handler.initialize_database(self.database)

    def _load_and_fit_classifier(self):
        x_train, y_train, _, _ = self.file_handler.load_database(self.current_folder, self.files_name, self.proportion)
        self.classifier.fit(x_train, y_train)

    def _validate_classifier(self):
        x_train, y_train, x_val, self.y_val = self.file_handler.load_database(self.current_folder, self.files_name, self.proportion)
        self.classifier.fit(x_train, y_train)
        self.y_predict, self.time_classifier = self.classifier.validate(x_val)
        self.target_names, _ = self.file_handler.initialize_database(self.database)
        self.file_handler.save_results(self.y_val, self.y_predict, self.time_classifier, self.target_names, os.path.join(self.current_folder, self.file_name_val))

    def _process_stage(self) -> None:
        if self.stage in [0, 1] and self.mode in ['D', 'RT']:
            success, frame = self._read_image()
            if not success:
                return
            if not self._image_processing(frame):
                return
            self._extract_features()
        elif self.stage == 2 and self.mode in ['D', 'RT']:
            self.process_reduction()
            if self.mode == 'D':
                self.stage = 3
            elif self.mode == 'RT':
                self.stage = 4
        elif self.stage == 3 and self.mode == 'D':
            if self._update_database():
                self.loop = False
            self.stage = 0
        elif self.stage == 4 and self.mode == 'RT':
            self._classify_gestures()
            self.stage = 0

    def _read_image_thread(self) -> None:
        while self.loop:
            success, frame = self.cap.read()
            if success:
                if frame.shape[0] != 640 or frame.shape[1] != 480:
                    frame = cv2.resize(frame, (640, 480))
                with self.frame_lock:
                    self.frame_captured = frame

    def _read_image(self) -> tuple[bool, np.ndarray]:
        with self.frame_lock:
            if self.frame_captured is None:
                return False, None
            frame = self.frame_captured.copy()  # Create a copy of the frame for thread-safe processing
        return True, frame

    def _image_processing(self, frame: np.ndarray) -> bool:
        try:
            results_people = self.tracking_processor.find_people(frame)
            results_identifies = self.tracking_processor.identify_operator(results_people)

            projected_window, bounding_box = self.tracking_processor.track_operator(results_people, results_identifies, frame)

            self.sps.check_person_centered(frame, bounding_box)

            self.hands_results, self.pose_results = self.feature.find_features(projected_window)

            frame_results = self.feature.draw_features(projected_window, self.hands_results, self.pose_results)

            if self.mode == 'D':
                cv2.putText(frame_results, f"S{self.stage} N{self.num_gest+1}: {self.y_val[self.num_gest]} D{self.dist_virtual_point:.3f}" , (25,25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 1, cv2.LINE_AA)
            elif self.mode == 'RT':
                cv2.putText(frame_results, f"S{self.stage} D{self.dist_virtual_point:.3f}" , (25,25), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 1, cv2.LINE_AA)
            cv2.imshow('RealSense Camera', frame_results)
            return True
        except Exception as e:
            print(f"E1 - Error during operator detection, tracking or feature extraction: {e}")
            with self.frame_lock:
                frame = self.frame_captured
            cv2.imshow('RealSense Camera', cv2.flip(frame, 1))
            self.hand_history = np.concatenate((self.hand_history, np.zeros((1, self.length, 42))))
            self.wrists_history = np.concatenate((self.wrists_history, np.zeros((1, self.length, 6))))
            self.stage = 0
            return False

    def _extract_features(self):
        self.hand_history, self.wrists_history = self.data_processor.update_data(self.hand_history, self.wrists_history, self.hands_results, self.pose_results)
        if self.data_processor.evaluate_data(self.hand_history[-1], self.wrists_history[-1], self.dist_virtual_point):
            self.stage = 2

    def process_reduction(self):
        self.sample = self.data_processor.extract_data(self.hand_history[-1], self.wrists_history[-1])
        self.sample = self.data_processor.reduce_data(self.sample)

    def _update_database(self) -> bool:
        self.data_processor.update_database(self.current_folder, self.file_name_build, self.sample, self.y_val[self.num_gest])
        self.num_gest += 1
        return self.num_gest == self.max_num_gest

    def _classify_gestures(self):
        gesture_id = self.classifier.classify(self.sample)
        gesture_name = self.gesture_analyzer.analyze_gesture(gesture_id, self.time_gesture, self.time_action)
        if gesture_name is not None:
            self.sps.apply_servo_positions(gesture_name)

    def _image_preview_thread(self):
        while self.loop:
            with self.frame_lock:
                if self.frame_captured is not None:
                    frame = self.frame_captured.copy()
            if frame is not None:
                cv2.imshow('RealSense Camera', cv2.flip(frame, 1))
                if cv2.waitKey(10) & 0xFF == ord("q"):
                    self.loop = False
                    break
        cv2.destroyAllWindows()

    def stop(self):
        """
        Signal the system to stop and wait for threads to exit.
        """
        self.loop = False
        self.image_thread.join()
        self.preview_thread.join()
        self.cap.release()
        cv2.destroyAllWindows()

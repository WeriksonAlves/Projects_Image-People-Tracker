#!/usr/bin/env python
# license removed for brevity
import numpy as np
from GestureEmotionCompiler import GestureEmotionCompiler
import rospy
from std_msgs.msg import Int16MultiArray, String

import os
from modules import *
from sklearn.neighbors import KNeighborsClassifier
import mediapipe as mp

database = {'F': [], 'I': [], 'L': [], 'P': [], 'T': []}
files_name= ['Datasets/DataBase_(5-10)_G.json',
            'Datasets/DataBase_(5-10)_H.json',
            'Datasets/DataBase_(5-10)_L.json',
            'Datasets/DataBase_(5-10)_M.json',
            'Datasets/DataBase_(5-10)_T.json',
            'Datasets/DataBase_(5-10)_1.json',
            'Datasets/DataBase_(5-10)_2.json',
            'Datasets/DataBase_(5-10)_3.json',
            'Datasets/DataBase_(5-10)_4.json',
            'Datasets/DataBase_(5-10)_5.json',
            'Datasets/DataBase_(5-10)_6.json',
            'Datasets/DataBase_(5-10)_7.json',
            'Datasets/DataBase_(5-10)_8.json',
            'Datasets/DataBase_(5-10)_9.json',
            'Datasets/DataBase_(5-10)_10.json'
            ]
real_time_mode = ModeFactory.create_mode('real_time', files_name=files_name, database=database)
mode = real_time_mode

model = GestureEmotionCompiler(
    model_name = "resnet18.onnx",
    model_option = "onnx",
    backend_option = 2, #1
    providers = 1,
    fp16 = False,
    num_faces = 1,
    
    config=InitializeConfig(4,30),
    operation=mode,
    file_handler=FileHandler(),
    current_folder=os.path.dirname(__file__),
    data_processor=DataProcessor(), 
    time_functions=TimeFunctions(), 
    gesture_analyzer=GestureAnalyzer(),
    tracking_processor=YoloProcessor('yolov8n-pose.pt'), 
    feature=HolisticProcessor(
        mp.solutions.hands.Hands(
            static_image_mode=False, 
            max_num_hands=1, 
            model_complexity=1, 
            min_detection_confidence=0.75, 
            min_tracking_confidence=0.75
            ),
        mp.solutions.pose.Pose(
            static_image_mode=False, 
            model_complexity=1, 
            smooth_landmarks=True, 
            enable_segmentation=False, 
            smooth_segmentation=True, 
            min_detection_confidence=0.75, 
            min_tracking_confidence=0.75
            )
        ),
    classifier=KNN(
        KNeighborsClassifier(
            n_neighbors=mode.k, 
            algorithm='auto', 
            weights='uniform'
            )
        )
    )


model.run()


G = GestureEmotionCompiler()
gesture = G.gesture_ros()
gesture_list = np.zeros(1, dtype = str) 

rospy.init_node('Gesture', anonymous=True)
pub = rospy.Publisher('/RosAria/gesture', String, queue_size=10)
gest = String()


def talker():

    rate = rospy.Rate(10) # 10hz
    while (gesture != 'F') or not rospy.is_shutdown():
        gesture_list.append(gesture)
        print(gesture)
            
        pub.publish(gest)
        rate.sleep()


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/RosAria/class", String, callback)


def update_gesture(msg : String, list):
    pass


def callback(data: String):
    gesture = data.data
    gesture_list.append(gesture)
    


if __name__ == '__main__':
    try:
        listener()
        talker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
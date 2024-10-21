# Onboard Servo-Visual Control for Action Recognition

This repository contains the project developed by Wérikson Alves in the course ELT791 - Special Topics - Robotics and its Utilities at the Federal University of Viçosa. The project focuses on servo-visual control coupled to a drone for recognizing human actions.

## Summary

- [Introduction](#introduction)
- [Objectives](#objectives)
- [Methodology](#Methodology)
  - [Camera Support](#camera-support)
  - [Gesture Recognition System](#gesture-recognition-system)
  - [ESP32CAM Configuration](#esp32cam-configuration)
  - [Recognition System with Camera Orientation](#recognition-system-with-camera-orientation)
- [Results Achieved](#results-achieved)
- [Final Considerations](#final-considerations)
- [Contact](#contact)

## Introduction
In this project, we tackle the challenges of human-robot interaction, especially in the context of gesture control using drones. The main goal is to keep the operator always in the drone's field of view without restricting the degrees of freedom, using an action recognition system and a servo-visual control for the camera.

## Objectives

- Keep the operator always in the drone's field of vision.
- Create an external module to attach the camera during missions.
- *Update the gesture recognition system to capture images from alternative sources.* (See other repository: [Gesture_Recognition_System](https://github.com/WeriksonAlves/Papers_CBA2024_Gesture_Recognition_System))
- Implement a servo-visual control system for the camera.

## Methodology

### Camera support

Implementation of a support for fixing the camera during flight missions.

### Gesture Recognition System

The system uses the YOLOv8 model for object detection and operator tracking, MediaPipe Hands for hand tracking, and BlazePose for body tracking. Gestures are classified using a kNN classifier.

### ESP32CAM configuration

Configuration of the ESP32CAM to transmit images via streaming over the Wi-Fi network. Implementation of OTA configuration for remote code update and servo motor control using ROS.

### Recognition System with Camera Orientation

Update of the recognition system to include new classes that handle ROS communication, servo initialization, and reading the transmitted images. Implementation of a proportional control routine to keep the operator in the center of the camera's field of view.

## Results Achieved

- **Image Acquisition:** Identification of limitations such as instability, slowness and distance from the access point.
- **ESP32CAM:** Evaluation of hardware limitations, especially in relation to transmission quality and frequency.
- **Support Module:** Effectiveness and challenges observed during the experiments.
- **Servo Motor Control:** Satisfactory performance with areas identified for future improvement.

## Final Considerations

The system developed is functional as a prototype, but requires improvement in areas such as image acquisition and ESP32CAM hardware limitations. The support and control of the servo motor showed good performance, indicating potential for future improvements.

## Contact

Wérikson F. de O. Alves  
Federal University of Viçosa 
Robotics Specialization Nucleus (NERo)  
Email: werikson.alves@ufv.br  
LinkedIn: [werikson-alves](https://www.linkedin.com/in/werikson-alves)  
YouTube: [Wérikson Alves](https://www.youtube.com/@weriksonalves5814)

---


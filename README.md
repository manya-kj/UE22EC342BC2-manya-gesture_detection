# **ROS2 Hand Gesture Recognition System**
A modular ROS2-based system for real-time hand gesture recognition, interpretation, and speech feedback.

## **Project Overview**
This project implements a complete pipeline for hand gesture recognition using ROS2 and MediaPipe. The system captures video from a camera, detects hand landmarks, interprets specific hand gestures, and provides audible feedback through text-to-speech conversion.
![Screenshot from 2025-05-16 19-00-12](https://github.com/user-attachments/assets/3845e1e8-af18-4c18-bfa5-85e85e724908)

## **Features**
1. Real-time hand landmark detection using MediaPipe
2. Recognition of 7 distinct hand gestures
3. Text-to-speech conversion of interpreted commands
4. Modular ROS2 node architecture for easy extension
5. Supports both online and offline text-to-speech engines
6. Visual feedback with landmark visualization

## **Recognized Gestures**
![Screenshot from 2025-05-16 19-03-50](https://github.com/user-attachments/assets/b540dd02-3592-4209-96ce-1067aa01098a)

## **Prerequisites**
- ROS2 (Foxy or later)
- Python 3.6+
- OpenCV
- MediaPipe
- pyttsx3 (primary TTS engine)
- gTTS and pygame (fallback TTS engine)

## **System Architecture**
The system consists of three interconnected ROS2 nodes:

1. Gesture Detector Node: Processes camera frames using MediaPipe to detect hand landmarks
2. Gesture Interpreter Node: Analyzes landmark data to recognize specific gestures
3. Speech Converter Node: Converts interpreted commands to speech

Project Structure:

![Screenshot from 2025-05-16 19-10-03](https://github.com/user-attachments/assets/af075344-eab2-4329-930b-2e74b4063a5c)

## **Usage**
1. Clone this repository into your ROS2 workspace:
```
cd ~/ros2_ws/src
git clone https://github.com/yourusername/ros2_gesture_recognition.git

```
2. Install dependencies:
```
pip install mediapipe opencv-python pyttsx3 gTTS pygame
```
3. Build the workspace:
```
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```
4. Launch the complete system
```
ros2 launch gesture_recognition gesture_system.launch.py
```

## **Results**

![Screenshot from 2025-05-16 19-12-34](https://github.com/user-attachments/assets/b75ccdc2-17e8-4e48-9f20-5488f49e87d0)
The screenshot above demonstrates the system's operation:

- Left panel: Visual feed showing a user performing the "Open Palm" gesture
- Right panel: Terminal output from the gesture_interpreter node
- The system correctly detects all five fingers extended (T:True I:True M:True R:True P:True)
- The console shows the detection process including thumb-wrist position measurements and angles
- Final output shows "Detected gesture: open_palm -> Stop" which triggers the speech output

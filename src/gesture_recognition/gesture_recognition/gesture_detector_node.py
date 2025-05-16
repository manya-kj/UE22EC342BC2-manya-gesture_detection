#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np
import time

class GestureDetectorNode(Node):
    def __init__(self):
        super().__init__('gesture_detector_node')
        
        # Initialize publishers and subscribers
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.hand_gesture_publisher = self.create_publisher(
            String, 
            '/gesture/hand_data', 
            10)
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize MediaPipe hands
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,  # Limit to one hand for more stable detection
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5)
        
        # For camera feed if no ROS2 camera topics available
        self.use_camera_directly = True
        if self.use_camera_directly:
            self.cap = cv2.VideoCapture(0)
            self.timer = self.create_timer(0.03, self.camera_timer_callback)
        
        # For debugging
        self.last_save_time = 0
        
        self.get_logger().info('Gesture Detector Node has started')

    def camera_timer_callback(self):
        """Process frames directly from camera when no ROS2 image topic is available"""
        ret, frame = self.cap.read()
        if ret:
            self.process_image(frame)
            
            # Display the image with hand landmarks
            cv2.imshow('Gesture Detection', frame)
            cv2.waitKey(1)

    def image_callback(self, msg):
        """Callback for processing images from ROS2 topic"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.process_image(cv_image)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def process_image(self, image):
        """Process image to detect hand landmarks and publish data"""
        # Mirror the image horizontally for more intuitive interaction
        image = cv2.flip(image, 1)
        
        # Convert BGR to RGB
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # Process the image and detect hands
        results = self.hands.process(image_rgb)
        
        # Draw hand landmarks on the image
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    image, 
                    hand_landmarks, 
                    self.mp_hands.HAND_CONNECTIONS)
                
                # Convert landmarks to a serializable format
                landmarks_data = []
                for landmark in hand_landmarks.landmark:
                    landmarks_data.append({
                        'x': landmark.x,
                        'y': landmark.y,
                        'z': landmark.z
                    })
                
                # Save debug image every 2 seconds
                current_time = time.time()
                if current_time - self.last_save_time > 2:
                    cv2.imwrite(f"/tmp/hand_detection_{int(current_time)}.jpg", image)
                    self.last_save_time = current_time
                
                # Add text to indicate hand is detected
                cv2.putText(image, "Hand Detected", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # Publish hand landmark data
                landmark_msg = String()
                landmark_msg.data = str(landmarks_data)
                self.hand_gesture_publisher.publish(landmark_msg)
        else:
            # Add text to indicate no hand is detected
            cv2.putText(image, "No Hand Detected", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    def __del__(self):
        """Clean up resources"""
        if self.use_camera_directly and hasattr(self, 'cap'):
            self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    gesture_detector = GestureDetectorNode()
    
    try:
        rclpy.spin(gesture_detector)
    except KeyboardInterrupt:
        pass
    finally:
        gesture_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

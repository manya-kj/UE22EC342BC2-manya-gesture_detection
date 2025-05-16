#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import ast
import math
import time

class GestureInterpreterNode(Node):
    def __init__(self):
        super().__init__('gesture_interpreter_node')
        
        # Create subscription for hand landmark data
        self.gesture_subscription = self.create_subscription(
            String,
            '/gesture/hand_data',
            self.gesture_callback,
            10)
        
        # Create publisher for interpreted text
        self.text_publisher = self.create_publisher(
            String,
            '/gesture/interpreted_text',
            10)
        
        # Define gesture mappings
        self.gestures = {
            'thumbs_up': 'Yes',
            'thumbs_down': 'No',
            'victory': 'Hello',
            'open_palm': 'Stop',
            'pointing': 'Go',  
            'ok_sign': 'OK',
            'phone': 'Call',  #phone gesture for "Call"
        }
        
        # Store previous gestures to avoid duplicate publishing
        self.last_detected_gesture = None
        self.gesture_stability_counter = 0
        self.stability_threshold = 3  # Reduced for faster response
        
        # For testing purposes
        self.test_mode = False  # Set to True to cycle through all gestures
        self.last_test_gesture_time = 0
        
        # Calibration values
        self.calibrated = False
        self.hand_size = 0.0
        
        # Store previous index finger position for tracking movement
        self.prev_index_tip = None
        self.index_movement_history = []
        self.movement_history_size = 5
        
        self.get_logger().info('Gesture Interpreter Node has started')

    def gesture_callback(self, msg):
        """Process hand landmark data and interpret gestures"""
        try:
            # Parse the hand landmarks from the message
            landmarks_data = ast.literal_eval(msg.data)
            
            # Test mode: cycle through gestures
            if self.test_mode:
                current_time = time.time()
                if current_time - self.last_test_gesture_time > 2.0:
                    test_idx = int((current_time / 2) % len(self.gestures))
                    test_gesture = list(self.gestures.keys())[test_idx]
                    gesture_text = self.gestures[test_gesture]
                    
                    self.get_logger().info(f'TEST MODE: {test_gesture} -> {gesture_text}')
                    
                    text_msg = String()
                    text_msg.data = gesture_text
                    self.text_publisher.publish(text_msg)
                    
                    self.last_test_gesture_time = current_time
                return

            # Calibrate if needed
            self.calibrate(landmarks_data)
            
            # Detect gesture based on the landmarks
            detected_gesture = self.detect_gesture(landmarks_data)
            
            # If a gesture is detected, check for stability
            if detected_gesture:
                if detected_gesture == self.last_detected_gesture:
                    self.gesture_stability_counter += 1
                else:
                    self.last_detected_gesture = detected_gesture
                    self.gesture_stability_counter = 1
                
                # Only publish when the gesture is stable
                if self.gesture_stability_counter >= self.stability_threshold:
                    gesture_text = self.gestures.get(detected_gesture, "Unknown gesture")
                    self.get_logger().info(f'Detected gesture: {detected_gesture} -> {gesture_text}')
                    
                    # Publish the interpreted text
                    text_msg = String()
                    text_msg.data = gesture_text
                    self.text_publisher.publish(text_msg)
                    
                    # Reset counter after publishing
                    self.gesture_stability_counter = 0
            
        except Exception as e:
            self.get_logger().error(f'Error interpreting gesture: {e}')

    def calibrate(self, landmarks):
        """Simple calibration to adapt to user's hand size and position"""
        if not self.calibrated and landmarks and len(landmarks) >= 21:
            # Calculate hand size based on wrist to middle finger distance
            wrist = landmarks[0]
            middle_tip = landmarks[12]
            self.hand_size = self.distance(wrist, middle_tip)
            self.calibrated = True
            self.get_logger().info(f"Calibrated hand size: {self.hand_size:.4f}")

    def detect_gesture(self, landmarks):
        """
        Detect gesture based on hand landmarks
        Returns the name of the detected gesture or None
        """
        # Extract key landmarks for gesture recognition
        if not landmarks or len(landmarks) < 21:
            return None
        
        # Get key landmarks
        thumb_tip = landmarks[4]
        index_tip = landmarks[8]
        middle_tip = landmarks[12]
        ring_tip = landmarks[16]
        pinky_tip = landmarks[20]
        wrist = landmarks[0]
        
        # Get knuckle landmarks
        thumb_mcp = landmarks[2]  # Thumb MCP joint
        thumb_ip = landmarks[3]   # Thumb IP joint (between knuckle and tip)
        index_pip = landmarks[6]  # Index PIP joint
        index_mcp = landmarks[5]  # Index MCP joint
        middle_pip = landmarks[10] # Middle PIP joint
        middle_mcp = landmarks[9]  # Middle MCP joint
        ring_pip = landmarks[14]   # Ring PIP joint
        pinky_pip = landmarks[18]  # Pinky PIP joint
        
        # Print raw landmarks for debugging
        self.get_logger().debug(f"WRIST: x:{wrist['x']:.3f} y:{wrist['y']:.3f} z:{wrist['z']:.3f}")
        self.get_logger().debug(f"THUMB: x:{thumb_tip['x']:.3f} y:{thumb_tip['y']:.3f} z:{thumb_tip['z']:.3f}")
        
        # Determine if fingers are extended
        # A finger is extended if its tip is higher (lower y value) than its base
        thumb_extended = thumb_tip['y'] < thumb_mcp['y']
        index_extended = index_tip['y'] < index_pip['y']
        middle_extended = middle_tip['y'] < middle_pip['y']
        ring_extended = ring_tip['y'] < ring_pip['y']
        pinky_extended = pinky_tip['y'] < pinky_pip['y']
        
        # Debug output for finger extension state
        self.get_logger().info(f"EXTENDED: T:{thumb_extended} I:{index_extended} M:{middle_extended} R:{ring_extended} P:{pinky_extended}")
        
        # Calculate relative position of thumb to wrist (for thumbs up/down)
        thumb_wrist_y_diff = wrist['y'] - thumb_tip['y']
        thumb_wrist_x_diff = thumb_tip['x'] - wrist['x']
        
        # Debug output for thumb position relative to wrist
        self.get_logger().info(f"THUMB-WRIST: y_diff:{thumb_wrist_y_diff:.3f} x_diff:{thumb_wrist_x_diff:.3f}")
        
        # Calculate the orientation of the thumb (angle relative to vertical)
        thumb_direction_y = thumb_tip['y'] - thumb_ip['y']
        thumb_direction_x = thumb_tip['x'] - thumb_ip['x']
        thumb_angle = math.degrees(math.atan2(thumb_direction_x, -thumb_direction_y))  # Negative y because y increases downward
        
        # Make sure the angle is between -180 and 180
        if thumb_angle > 180:
            thumb_angle -= 360
        elif thumb_angle < -180:
            thumb_angle += 360
            
        self.get_logger().info(f"THUMB ANGLE: {thumb_angle:.1f} degrees")
        
        # Distance between thumb and index tip (for OK sign)
        thumb_index_distance = self.distance(thumb_tip, index_tip)
        self.get_logger().info(f"THUMB-INDEX DISTANCE: {thumb_index_distance:.3f}")
        
        # Track index finger movement for pointing forward
        if self.prev_index_tip is not None:
            # Calculate the z-movement (toward/away from camera)
            z_movement = index_tip['z'] - self.prev_index_tip['z']
            
            # Store z-movement in history
            self.index_movement_history.append(z_movement)
            if len(self.index_movement_history) > self.movement_history_size:
                self.index_movement_history.pop(0)
        
        # Update previous index tip position
        self.prev_index_tip = index_tip.copy()
        
        # Thumbs up: thumb is extended upward, other fingers curled
        # The thumb should be pointing upward (angle close to 0 degrees)
        if (not index_extended and not middle_extended and not ring_extended and not pinky_extended and
            thumb_extended and thumb_wrist_y_diff > 0.1 and abs(thumb_angle) < 45):
            self.get_logger().info("Detected: THUMBS UP")
            return 'thumbs_up'
        
        # Thumbs down: thumb is extended downward, other fingers curled
        # The thumb should be pointing downward (angle close to 180 degrees)
        if (not index_extended and not middle_extended and not ring_extended and not pinky_extended and
            thumb_wrist_y_diff < -0.1 and abs(thumb_angle) > 135):
            self.get_logger().info("Detected: THUMBS DOWN")
            return 'thumbs_down'
        
        # Victory: index and middle extended, others closed
        if index_extended and middle_extended and not ring_extended and not pinky_extended:
            # Check that index and middle fingers are separated (not together)
            index_middle_distance = self.distance(index_tip, middle_tip)
            if index_middle_distance > 0.05:  # Threshold for separation
                self.get_logger().info("Detected: VICTORY")
                return 'victory'
        
        # Pointing gesture for 'Go': only index extended
        if index_extended and not middle_extended and not ring_extended and not pinky_extended:
            # Check if the finger is pointing in any direction (no specific direction required)
            self.get_logger().info("Detected: POINTING (GO)")
            return 'pointing'
        
        # OK sign: thumb and index tips are close to each other, forming a circle
        if thumb_index_distance < 0.1:
            self.get_logger().info("Detected: OK SIGN")
            return 'ok_sign'
        
        # Open palm: all fingers extended
        if thumb_extended and index_extended and middle_extended and ring_extended and pinky_extended:
            self.get_logger().info("Detected: OPEN PALM")
            return 'open_palm'
        
        # Phone/Call gesture: thumb and pinky extended, other fingers closed
        if thumb_extended and not index_extended and not middle_extended and not ring_extended and pinky_extended:
            self.get_logger().info("Detected: PHONE CALL")
            return 'phone'
        
        # Closed fist: no fingers extended - not mapped to any gesture
        if not thumb_extended and not index_extended and not middle_extended and not ring_extended and not pinky_extended:
            # Make sure the hand is actually in view and not just at the edge
            if self.all_fingers_visible(landmarks):
                self.get_logger().info("Detected: CLOSED FIST (not mapped)")
                return None
        
        self.get_logger().info("No gesture detected")
        return None

    def calculate_finger_direction(self, landmarks, tip_idx, base_idx):
        """Calculate the direction vector of a finger from base to tip"""
        tip = landmarks[tip_idx]
        base = landmarks[base_idx]
        
        # Calculate direction vector
        direction = {
            'x': tip['x'] - base['x'],
            'y': tip['y'] - base['y'],
            'z': tip['z'] - base['z']
        }
        
        # Normalize the vector
        magnitude = math.sqrt(direction['x']**2 + direction['y']**2 + direction['z']**2)
        if magnitude > 0:
            direction['x'] /= magnitude
            direction['y'] /= magnitude
            direction['z'] /= magnitude
        
        self.get_logger().info(f"FINGER DIRECTION: x:{direction['x']:.3f} y:{direction['y']:.3f} z:{direction['z']:.3f}")
        return direction

    def all_fingers_visible(self, landmarks):
        """Check if all fingers are visible in the frame"""
        # Calculate average z position of landmarks
        z_values = [landmark['z'] for landmark in landmarks]
        avg_z = sum(z_values) / len(z_values)
        
        # Check that the z values are not too spread out
        z_variance = sum((z - avg_z) ** 2 for z in z_values) / len(z_values)
        return z_variance < 0.01  # Threshold based on testing
    
    def distance(self, point1, point2):
        """Calculate Euclidean distance between two 3D points"""
        return math.sqrt(
            (point1['x'] - point2['x'])**2 + 
            (point1['y'] - point2['y'])**2 + 
            (point1['z'] - point2['z'])**2
        )

def main(args=None):
    rclpy.init(args=args)
    gesture_interpreter = GestureInterpreterNode()
    
    try:
        rclpy.spin(gesture_interpreter)
    except KeyboardInterrupt:
        pass
    finally:
        gesture_interpreter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

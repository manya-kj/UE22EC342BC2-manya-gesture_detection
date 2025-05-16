from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gesture_recognition',
            executable='gesture_detector',
            name='gesture_detector',
            output='screen'
        ),
        Node(
            package='gesture_recognition',
            executable='gesture_interpreter',
            name='gesture_interpreter',
            output='screen'
        ),
        Node(
            package='gesture_recognition',
            executable='speech_converter',
            name='speech_converter',
            output='screen'
        ),
    ])

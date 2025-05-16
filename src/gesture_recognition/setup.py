from setuptools import setup
import os
from glob import glob

package_name = 'gesture_recognition'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS2 package for gesture recognition and conversion to text/speech',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesture_detector = gesture_recognition.gesture_detector_node:main',
            'gesture_interpreter = gesture_recognition.gesture_interpreter_node:main',
            'speech_converter = gesture_recognition.speech_converter_node:main',
        ],
    },
)

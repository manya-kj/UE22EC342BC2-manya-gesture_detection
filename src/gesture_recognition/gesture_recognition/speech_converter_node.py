#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import time
import tempfile

try:
    import pyttsx3
    USE_PYTTSX3 = True
except ImportError:
    USE_PYTTSX3 = False
    from gtts import gTTS
    import pygame

class SpeechConverterNode(Node):
    def __init__(self):
        super().__init__('speech_converter_node')
        
        # Create subscription to the interpreted text
        self.text_subscription = self.create_subscription(
            String,
            '/gesture/interpreted_text',
            self.text_callback,
            10)
        
        # Initialize audio system based on available libraries
        if USE_PYTTSX3:
            self.engine = pyttsx3.init()
            self.engine.setProperty('rate', 150)  # Speaking rate
            self.engine.setProperty('volume', 0.9)  # Volume 0-1
            self.get_logger().info('Using pyttsx3 for speech synthesis')
        else:
            # Initialize pygame for audio playback with gTTS
            pygame.mixer.init()
            
            # Create a temporary directory to store audio files
            self.temp_dir = tempfile.mkdtemp()
            self.get_logger().info(f'Using gTTS with temp directory: {self.temp_dir}')
            
            # Cache to avoid regenerating speech for the same text
            self.speech_cache = {}
        
        # Wait time between consecutive speech outputs
        self.min_speech_interval = 1.5  # seconds (reduced for better responsiveness)
        self.last_speech_time = 0
        
        self.get_logger().info('Speech Converter Node has started')

    def text_callback(self, msg):
        """Convert text to speech and play it"""
        text = msg.data
        
        # Skip if the text is empty
        if not text:
            return
            
        self.get_logger().info(f'Converting to speech: {text}')
        
        # Check if enough time has passed since the last speech
        current_time = time.time()
        if current_time - self.last_speech_time < self.min_speech_interval:
            self.get_logger().info('Speech too frequent, skipping this one')
            return
        
        try:
            if USE_PYTTSX3:
                # Use pyttsx3 (offline TTS)
                self.engine.say(text)
                self.engine.runAndWait()
            else:
                # Use gTTS (online TTS)
                if text in self.speech_cache:
                    audio_file = self.speech_cache[text]
                else:
                    # Generate a unique filename
                    audio_file = os.path.join(self.temp_dir, f"speech_{hash(text)}.mp3")
                    
                    # Generate speech using gTTS
                    tts = gTTS(text=text, lang='en')
                    tts.save(audio_file)
                    
                    # Cache the filename
                    self.speech_cache[text] = audio_file
                
                # Play the audio
                pygame.mixer.music.load(audio_file)
                pygame.mixer.music.play()
                
                # Wait for audio to finish playing
                while pygame.mixer.music.get_busy():
                    pygame.time.Clock().tick(10)
            
            # Update last speech time
            self.last_speech_time = current_time
            
        except Exception as e:
            self.get_logger().error(f'Error converting text to speech: {e}')

    def __del__(self):
        """Clean up resources"""
        if not USE_PYTTSX3:
            # Clean up temporary files for gTTS
            try:
                for audio_file in self.speech_cache.values():
                    if os.path.exists(audio_file):
                        os.remove(audio_file)
                os.rmdir(self.temp_dir)
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    speech_converter = SpeechConverterNode()
    
    try:
        rclpy.spin(speech_converter)
    except KeyboardInterrupt:
        pass
    finally:
        speech_converter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

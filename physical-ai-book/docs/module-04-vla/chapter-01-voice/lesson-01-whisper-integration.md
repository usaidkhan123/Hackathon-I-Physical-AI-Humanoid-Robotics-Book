---
id: lesson-01-whisper-integration
title: "Lesson 1: Whisper Integration for Speech-to-Text"
sidebar_position: 1
description: Integrate OpenAI's Whisper model for accurate speech-to-text conversion in robotics.
---

# Lesson 1: Whisper Integration for Speech-to-Text

## Lesson Objectives
- Understand the basics of speech-to-text (STT) technology.
- Learn how to integrate OpenAI's Whisper model into a Python application.
- Develop a basic ROS 2 node for capturing audio and converting it to text using Whisper.

## Prerequisites
- Basic understanding of Python.
- ROS 2 Foxy/Humble installed.
- Access to OpenAI API key (for cloud Whisper API, or local installation if using local Whisper).
- Basic microphone setup on your development machine/robot.

## Concept Explanation
Speech-to-Text (STT) is the process of converting spoken language into written text. OpenAI's Whisper is a general-purpose speech recognition model that demonstrates robust performance across various languages and acoustic environments. Its integration allows robots to process human voice commands accurately, a fundamental step in Vision-Language-Action (VLA) systems.

## Step-by-Step Technical Breakdown

1.  **Install Whisper**:
    If using the OpenAI API, install the `openai` Python package. If using local Whisper, install `openai-whisper` and `pytorch`.
    ```bash
    pip install openai # For API
    # OR
    pip install -U openai-whisper
    # Ensure you have a C++ compiler for certain dependencies if installing locally.
    ```
2.  **Audio Capture (ROS 2 Node)**:
    Create a ROS 2 Python node that listens to audio input from a microphone. This node will capture short audio snippets.
3.  **Whisper Processing**:
    Pass the captured audio snippet to the Whisper model (either via API or locally) to get the transcribed text.
4.  **Publish Text**:
    Publish the transcribed text on a ROS 2 topic (e.g., `/voice_command_text`).

## Real-World Analogy
Imagine you have a personal assistant who can listen to what you say and write it down perfectly, even if you speak quickly or have an accent. Whisper is like that assistant for your robot. Before your robot can understand what you *mean*, it first needs to know *what you said*.

## Hands-On Tasks
1.  Set up your microphone and verify it's working with a simple Python script.
2.  Write a Python script to record a 5-second audio clip and save it to a WAV file.
3.  Integrate Whisper to transcribe the WAV file.
4.  Create a ROS 2 `speech_to_text_node.py` that continuously records audio, transcribes it with Whisper, and publishes the text.

## Python + ROS2 Examples

```python
# speech_to_text_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sounddevice as sd
import numpy as np
import wavio
import openai # or import whisper if local

class SpeechToTextNode(Node):
    def __init__(self):
        super().__init__('speech_to_text_node')
        self.publisher_ = self.create_publisher(String, 'voice_command_text', 10)
        self.timer_period = 5.0 # seconds for each recording
        self.timer = self.create_timer(self.timer_period, self.record_and_transcribe)

        self.fs = 16000 # Sample rate for audio
        self.recording = None
        self.get_logger().info('Speech-to-Text Node started. Waiting for voice commands...')

    def record_audio(self):
        self.get_logger().info('Recording audio...')
        self.recording = sd.rec(int(self.timer_period * self.fs), samplerate=self.fs, channels=1)
        sd.wait()
        self.get_logger().info('Recording finished.')
        # Save to a temporary WAV file
        wavio.write("temp_audio.wav", self.recording, self.fs, sampwidth=2)
        return "temp_audio.wav"

    def transcribe_audio(self, audio_file_path):
        try:
            # Using OpenAI API (replace with local model if preferred)
            # Make sure to set your OpenAI API key in your environment variables
            with open(audio_file_path, "rb") as audio_file:
                response = openai.audio.transcriptions.create(
                    model="whisper-1",
                    file=audio_file
                )
            return response.text
        except Exception as e:
            self.get_logger().error(f"Error transcribing audio: {e}")
            return ""

    def record_and_transcribe(self):
        audio_file = self.record_audio()
        if audio_file:
            transcribed_text = self.transcribe_audio(audio_file)
            if transcribed_text:
                msg = String()
                msg.data = transcribed_text
                self.publisher_.publish(msg)
                self.get_logger().info(f'Published: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    speech_to_text_node = SpeechToTextNode()
    rclpy.spin(speech_to_text_node)
    speech_to_text_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Debugging Tips
-   **Microphone Issues**: Ensure your microphone is correctly recognized by your operating system. Use tools like `arecord -L` (Linux) or check Sound settings (Windows/macOS).
-   **API Key Errors**: Double-check your OpenAI API key and ensure it's correctly set as an environment variable or passed in your code.
-   **Audio Format**: Whisper expects specific audio formats (e.g., WAV, MP3). Ensure your recording saves in a compatible format and sample rate (16kHz recommended).
-   **Network Latency**: If using the cloud Whisper API, network issues can cause delays. For faster response, consider local Whisper models or optimized audio chunks.

## Mini Quiz (4-6 questions)
1.  What is the primary function of a Speech-to-Text (STT) system in VLA robotics?
2.  Which OpenAI model is commonly used for STT conversion?
3.  Why is a ROS 2 topic a suitable mechanism for publishing transcribed voice commands?
4.  Name two common challenges in integrating STT systems with robotic applications.
5.  What is the purpose of `sd.rec` in the provided Python example?

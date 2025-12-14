---
id: lesson-03-wake-word-detection
title: "Lesson 3: Wake Word Detection"
sidebar_position: 3
description: Implement wake word detection to activate the robot's listening mode efficiently.
---

# Lesson 3: Wake Word Detection

## Lesson Objectives
- Understand the concept and importance of wake word detection in conversational AI.
- Learn how to use a wake word engine (e.g., Picovoice Porcupine) to trigger speech-to-text.
- Integrate wake word detection into a ROS 2 system to activate voice command processing on demand.

<h2>Prerequisites</h2>
- Completion of Lesson 1: Whisper Integration and Lesson 2: Audio Preprocessing.
- Basic understanding of event-driven programming.
- Account with a wake word provider (e.g., Picovoice for Porcupine).

<h2>Concept Explanation</h2>
Continuously processing all audio for speech commands is computationally expensive and inefficient. Wake word detection allows a system to remain in a low-power listening state until a specific phrase (the "wake word," e.g., "Hey Robot," "Computer") is spoken. Only then does the full STT system activate, significantly improving efficiency and user experience by preventing unintended commands.

<h2>Step-by-Step Technical Breakdown</h2>
1.  **Choose a Wake Word Engine**: Select an appropriate engine (e.g., Picovoice Porcupine, OpenWakeWord, or a custom solution). For this lesson, we will assume a basic integration with a pre-trained model.
2.  **Obtain Access Key/Model**: Register with the provider (if cloud-based) or download necessary models.
3.  **Integrate with Audio Stream**: The wake word engine continuously monitors the audio stream, typically on a separate thread or process to avoid blocking.
4.  **Trigger STT**: Once the wake word is detected, an event is triggered to start the more resource-intensive STT process (Whisper).
5.  **ROS 2 Integration**: Create a ROS 2 node that runs the wake word engine. This node will publish a signal (e.g., a boolean message or a custom message type) on a ROS 2 topic when the wake word is detected. The STT node then subscribes to this topic and activates.

<h2>Real-World Analogy</h2>
Think of a guard dog that sleeps until someone says a special command, and *then* it wakes up and pays attention. Wake word detection is like that special command for your robot; it tells the robot, "Okay, now listen carefully, I have something important to say."

<h2>Hands-On Tasks</h2>
1.  Sign up for a free Picovoice account and obtain your AccessKey.
2.  Install the Porcupine Python package (`pip install pvporcupine`).
3.  Write a simple Python script to detect a custom wake word using Porcupine.
4.  Modify your ROS 2 `speech_to_text_node.py` from Lesson 1 to only start transcribing after a wake word is detected. Implement a separate node for wake word detection, or integrate it as a separate thread.

<h2>Python + ROS2 Examples</h2>

```python
# wake_word_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import pvporcupine
import pyaudio
import struct

# Replace with your Picovoice AccessKey
PICOVOICE_ACCESS_KEY = "YOUR_PICOVOICE_ACCESS_KEY" 
# Path to your wake word model (.ppn file)
# You can generate custom wake words at Picovoice Console
WAKE_WORD_MODEL_PATH = "path/to/your/custom_wake_word.ppn" 

class WakeWordNode(Node):
    def __init__(self):
        super().__init__('wake_word_node')
        self.publisher_ = self.create_publisher(Bool, 'wake_word_detected', 10)
        
        try:
            self.porcupine = pvporcupine.create(
                access_key=PICOVOICE_ACCESS_KEY,
                keyword_paths=[WAKE_WORD_MODEL_PATH]
            )
        except pvporcupine.PorcupineError as e:
            self.get_logger().error(f"Failed to initialize Porcupine: {e}")
            self.get_logger().error("Ensure your AccessKey and wake word model path are correct.")
            self.porcupine = None
            return

        self.py_audio = pyaudio.PyAudio()
        self.audio_stream = self.py_audio.open(
            rate=self.porcupine.sample_rate,
            channels=1,
            format=pyaudio.paInt16,
            input=True,
            frames_per_buffer=self.porcupine.frame_length
        )

        self.get_logger().info('Wake Word Node started. Listening for wake word...')
        self.create_timer(0.01, self.listen_for_wake_word) # Check audio stream frequently

    def listen_for_wake_word(self):
        if self.porcupine is None:
            return

        pcm = self.audio_stream.read(self.porcupine.frame_length, exception_on_overflow=False)
        pcm = struct.unpack_from("h" * self.porcupine.frame_length, pcm)

        keyword_index = self.porcupine.process(pcm)

        if keyword_index >= 0:
            self.get_logger().info(f"Wake word detected!")
            msg = Bool()
            msg.data = True
            self.publisher_.publish(msg)

    def destroy_node(self):
        if self.porcupine is not None:
            self.porcupine.delete()
        if self.audio_stream is not None:
            self.audio_stream.close()
        if self.py_audio is not None:
            self.py_audio.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    wake_word_node = WakeWordNode()
    rclpy.spin(wake_word_node)
    # The destroy_node method will be called when rclpy.spin returns
    # or the node is explicitly destroyed.

if __name__ == '__main__':
    main()
```

<h2>Debugging Tips</h2>
-   **AccessKey/Model Path**: Verify your Picovoice AccessKey and the path to your `.ppn` wake word model. Incorrect values will prevent initialization.
-   **Audio Input**: Ensure the correct microphone is selected and providing input to `pyaudio`.
-   **Resource Usage**: Monitor CPU usage. While wake word detection is generally light, issues with audio buffers or processing loops can spike CPU.
-   **False Positives/Negatives**: Adjust sensitivity settings if available in your wake word engine. Retraining models or choosing a more distinct wake word can help.

<h2>Mini Quiz (4-6 questions)</h2>
1.  What problem does wake word detection solve in VLA systems?
2.  Why is continuous STT processing inefficient?
3.  How does a wake word engine typically communicate detection to other parts of a robotic system in ROS 2?
4.  What is a common open-source library for wake word detection?
5.  What are the consequences of an overly sensitive wake word detector?

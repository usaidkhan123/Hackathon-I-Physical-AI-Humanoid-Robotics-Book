---
id: lesson-01-intent-detection
title: "Lesson 1: Intent Detection with LLMs"
sidebar_position: 1
description: "Utilize Large Language Models (LLMs) to accurately identify the user's intended action from natural language commands."
---

# Lesson 1: Intent Detection with LLMs

## Lesson Objectives
- Understand the role of intent detection in natural language processing for robotics.
- Learn how to structure prompts for LLMs to extract user intent.
- Implement an LLM-based intent detection module in Python and integrate it with ROS 2.

## Prerequisites
- Completion of Chapter 1 lessons on Voice Perception.
- Basic understanding of Large Language Models and API interaction (e.g., OpenAI API).
- Python programming skills.

## Concept Explanation
After converting speech to text, the next crucial step is to understand the user's intention. Intent detection involves classifying the user's command into predefined categories of actions the robot can perform (e.g., "navigate," "pick_up," "report_status"). Large Language Models excel at this by leveraging their vast knowledge to interpret nuances in language, making them powerful tools for robust intent recognition in robotics.

<h2>Step-by-Step Technical Breakdown</h2>

1.  **Define Robot Capabilities (Intents)**: List all high-level actions your robot can perform (e.g., `move_to_location`, `grasp_object`, `identify_object`, `report_battery`).
2.  **Prompt Engineering for Intent**: Design an LLM prompt that clearly instructs the model to identify the user's intent from a given text command, optionally extracting relevant parameters (slots).
3.  **LLM API Interaction**: Send the user's transcribed text along with your engineered prompt to an LLM (e.g., OpenAI GPT series) and parse its response.
4.  **ROS 2 Integration**: Create a ROS 2 node that subscribes to the `/voice_command_text` topic, processes the text with the LLM for intent, and publishes the detected intent (e.g., on a `/robot_intent` topic).

<h2>Real-World Analogy</h2>
Imagine a busy restaurant where a waiter (robot) hears a customer's order (voice command). Before serving, the waiter needs to understand if the customer wants to "order food," "ask for the bill," or "complain about the service." Intent detection is like the waiter quickly figuring out the main purpose of the customer's speech.

<h2>Hands-On Tasks</h2>
1.  Define a JSON schema for your robot's intents and their required parameters.
2.  Write a Python function that takes a text command and uses the OpenAI API to call an LLM for intent detection, returning the intent and any extracted parameters.
3.  Create a ROS 2 `intent_detector_node.py` that subscribes to `/voice_command_text`, uses your Python function, and publishes a custom `IntentMessage` (ROS 2 custom message type) on `/robot_intent`.

<h2>Python + ROS2 Examples</h2>

```python
# intent_detector_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# Assuming a custom ROS 2 message type for Intent, e.g., robot_msgs/msg/Intent.idl
# For simplicity, we'll use a String message for now, publishing JSON.
# from robot_msgs.msg import Intent 
import openai
import json
import os

class IntentDetectorNode(Node):
    def __init__(self):
        super().__init__('intent_detector_node')
        self.subscription = self.create_subscription(
            String,
            'voice_command_text',
            self.listener_callback,
            10
        )
        self.publisher_ = self.create_publisher(String, 'robot_intent', 10) # Publishing JSON string
        self.get_logger().info('Intent Detector Node started. Waiting for text commands...')

        self.openai_client = openai.OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))

    def get_llm_response(self, text_command):
        # Example: Define the structure of your robot's capabilities
        # This could also come from a dynamically loaded capabilities file
        robot_capabilities = {
            "move_to_location": {
                "description": "Move the robot to a specified location.",
                "parameters": {"location": {"type": "string", "enum": ["kitchen", "living_room", "bedroom"]}}
            },
            "grasp_object": {
                "description": "Pick up a specified object.",
                "parameters": {"object": {"type": "string", "enum": ["apple", "cup", "book"]}}
            },
            "report_battery": {
                "description": "Report the current battery level.",
                "parameters": {}
            }
        }

        prompt = f"""
        You are a robot command interpreter. Your goal is to extract the user's intent and any relevant parameters from their command.
        The robot can perform the following actions:
        {json.dumps(robot_capabilities, indent=2)}

        User command: "{text_command}"

        Identify the most likely intent and extract all parameters. Respond with a JSON object in the following format:
        {{
            "intent": "identified_intent",
            "parameters": {{ "param_name": "param_value" }}
        }}
        If no clear intent is found, use "unknown".
        """

        try:
            response = self.openai_client.chat.completions.create(
                model="gpt-4o-mini", # Or another suitable model
                messages=[
                    {"role": "system", "content": prompt},
                    {"role": "user", "content": text_command}
                ],
                response_format={"type": "json_object"}
            )
            return json.loads(response.choices[0].message.content)
        except Exception as e:
            self.get_logger().error(f"LLM API call failed: {e}")
            return {"intent": "error", "parameters": {"details": str(e)}}

    def listener_callback(self, msg):
        self.get_logger().info(f'Received text: "{msg.data}"')
        llm_output = self.get_llm_response(msg.data)
        
        intent_msg = String() # Using String to publish JSON for simplicity
        intent_msg.data = json.dumps(llm_output)
        self.publisher_.publish(intent_msg)
        self.get_logger().info(f'Published intent: "{intent_msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    intent_detector_node = IntentDetectorNode()
    rclpy.spin(intent_detector_node)
    intent_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Debugging Tips
-   **Prompt Quality**: If the LLM is not identifying intents correctly, refine your prompt. Be explicit about the desired output format and available actions.
-   **API Key/Connectivity**: Ensure your OpenAI API key is valid and your system has internet connectivity.
-   **JSON Parsing**: Verify that the LLM's response is valid JSON and that your parsing logic is robust to minor variations.
-   **Intent Overlap**: If multiple intents are too similar, the LLM might struggle to differentiate. Make your robot's capabilities distinct.

## Mini Quiz (4-6 questions)
1.  What is the primary purpose of intent detection in a VLA system?
2.  How do LLMs assist in the process of intent detection?
3.  What are "slots" in the context of intent detection?
4.  Why is it important to define your robot's capabilities clearly for the LLM?
5.  What ROS 2 message type might be suitable for publishing detected intents with parameters?

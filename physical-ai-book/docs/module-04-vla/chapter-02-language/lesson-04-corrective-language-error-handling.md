---
id: lesson-04-corrective-language-error-handling
title: "Lesson 4: Corrective Language and Error Handling"
sidebar_position: 4
description: "Design LLM interactions to handle errors, ask clarifying questions, and adapt to corrective human language."
---

# Lesson 4: Corrective Language and Error Handling

## Lesson Objectives
- Understand the challenges of error handling and ambiguity in natural language commands for robots.
- Learn to design LLM prompts that enable the robot to ask clarifying questions.
- Implement mechanisms for the robot to adapt its plan based on human corrective language.

## Prerequisites
- Completion of previous lessons in this chapter.
- Basic understanding of conversational turns and state management in an interactive system.

## Concept Explanation
Human language is inherently ambiguous and prone to error. Robots need to do more than just execute commands; they must intelligently handle situations where commands are unclear, unfeasible, or incorrect. This involves using LLMs to generate clarifying questions, acknowledging errors, and dynamically adjusting task plans based on human feedback or corrective language. Effective error handling is crucial for creating robust, user-friendly VLA systems that can gracefully recover from misunderstandings.

<h2>Step-by-Step Technical Breakdown</h2>
1.  **Error Detection**: Identify conditions under which a command might be problematic (e.g., ambiguous entities, unfeasible actions detected by robot's internal state, LLM failing to generate a coherent plan).
2.  **Clarification Prompts**: Engineer LLM prompts to generate specific, disambiguating questions when an error condition is met.
3.  **Context Management**: Maintain conversational context so the LLM can understand follow-up questions and corrections (e.g., "No, not the red one, not the *blue* one"). This often involves sending a history of recent turns to the LLM.
4.  **Plan Adaptation**: Implement logic to re-prompt the LLM or modify the existing plan based on human clarification or correction.

<h2>Real-World Analogy</h2>
Imagine you ask someone to "bring the book." If there are multiple books, a human would instinctively ask, "Which book?" If you then say, "The red one," they understand the correction. This lesson teaches your robot to have that same kind of intelligent, error-recovering conversation.

<h2>Hands-On Tasks</h2>
1.  Modify your LLM interaction code to detect when an entity is ambiguous (e.g., if the LLM cannot confidently identify a single object from its perception or its knowledge base).
2.  When ambiguity is detected, craft an LLM prompt that asks for clarification (e.g., "There are multiple books. Which one did you mean?").
3.  Implement a simple conversational state machine to handle a two-turn exchange for clarification.
4.  (Advanced) Experiment with passing a short conversational history to the LLM to enable more natural corrections.

<h2>Python + ROS2 Examples</h2>

```python
# error_handling_llm_node.py (conceptual example, building on previous lessons)
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import json
import os
import collections

class ErrorHandlingLLMNode(Node):
    def __init__(self):
        super().__init__('error_handling_llm_node')
        self.subscription_text = self.create_subscription(
            String,
            'voice_command_text',
            self.text_command_callback,
            10
        )
        self.publisher_response = self.create_publisher(String, 'robot_response_speech', 10) # For robot to speak
        self.publisher_plan = self.create_publisher(String, 'robot_task_plan', 10)
        self.get_logger().info('Error Handling LLM Node started. Waiting for commands...')

        self.openai_client = openai.OpenAI(api_key=os.environ.get("OPENAI_API_KEY"))
        self.conversation_history = collections.deque(maxlen=5) # Store last 5 turns

    def get_llm_response_with_error_handling(self, current_command):
        # Simulate an ambiguous scenario:
        ambiguous_entities = ["book", "cup"] 
        detected_ambiguity = False
        for entity in ambiguous_entities:
            if entity in current_command.lower() and current_command.lower().count(entity) > 1:
                detected_ambiguity = True
                break

        system_message = {
            "role": "system",
            "content": f"""
            You are a helpful robot assistant. You need to process human commands.
            If a command is ambiguous or causes an error, you must ask a clarifying question.
            Your response should be in JSON format:
            {{
                "action": "plan" | "clarify" | "respond",
                "message": "text to say or question to ask",
                "plan": [{{...}}] // Only if action is "plan"
            }}
            
            Current conversation history: {list(self.conversation_history)}
            """
        }

        user_message = {"role": "user", "content": current_command}
        messages = [system_message, user_message]
        
        # Add history to context
        for turn in self.conversation_history:
            messages.append({"role": "user", "content": turn["user_utterance"]})
            messages.append({"role": "assistant", "content": turn["robot_response"]})

        try:
            response = self.openai_client.chat.completions.create(
                model="gpt-4o-mini",
                messages=messages,
                response_format={"type": "json_object"}
            )
            llm_output = json.loads(response.choices[0].message.content)
            return llm_output
        except Exception as e:
            self.get_logger().error(f"LLM API call for error handling failed: {e}")
            return {"action": "respond", "message": "I am sorry, I encountered an internal error. Please try again."}

    def text_command_callback(self, msg):
        self.get_logger().info(f'Received command: "{msg.data}"')
        
        # Get LLM response, potentially with clarification
        llm_processed_output = self.get_llm_response_with_error_handling(msg.data)
        
        # Update conversation history
        self.conversation_history.append({
            "user_utterance": msg.data,
            "robot_response": llm_processed_output.get("message", json.dumps(llm_processed_output.get("plan", []))) # Store what robot would "say" or "do"
        })

        if llm_processed_output.get("action") == "clarify":
            robot_speech_msg = String()
            robot_speech_msg.data = llm_processed_output.get("message", "Can you please clarify?")
            self.publisher_response.publish(robot_speech_msg)
            self.get_logger().info(f'Robot asks for clarification: "{robot_speech_msg.data}"')
        elif llm_processed_output.get("action") == "plan":
            plan_msg = String()
            plan_msg.data = json.dumps(llm_processed_output.get("plan", []))
            self.publisher_plan.publish(plan_msg)
            robot_speech_msg = String()
            robot_speech_msg.data = f"Executing plan: {llm_processed_output.get('message', 'Command understood.')}"
            self.publisher_response.publish(robot_speech_msg)
            self.get_logger().info(f'Robot plans and responds: "{robot_speech_msg.data}"')
        else: # "respond" or fallback
            robot_speech_msg = String()
            robot_speech_msg.data = llm_processed_output.get("message", "I didn't quite understand that. Can you rephrase?")
            self.publisher_response.publish(robot_speech_msg)
            self.get_logger().info(f'Robot responds: "{robot_speech_msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    error_handling_llm_node = ErrorHandlingLLMNode()
    rclpy.spin(error_handling_llm_node)
    error_handling_llm_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Debugging Tips
-   **Conversation History**: Ensure your `conversation_history` is correctly managed and passed to the LLM. Incorrect context can lead to the LLM forgetting previous turns or clarifications.
-   **Clear Instructions**: Be very precise in your system message to the LLM about *when* to clarify and *how* to format its clarifying questions.
-   **Testing Edge Cases**: Actively test with ambiguous commands, unfeasible requests, and direct corrections to see how the system responds.
-   **Robot Internal State**: For more advanced error handling, feed the robot's current internal state (e.g., object detection results, navigation status) into the LLM's context.

<h2>Mini Quiz (4-6 questions)</h2>
1.  Why is error handling crucial for human-robot interaction using natural language?
2.  How can LLMs be used to ask clarifying questions?
3.  What role does "conversational context" play in handling corrective language?
4.  When might a robot need to ask for clarification from a human?
5.  What is a major challenge when dealing with ambiguity in spoken commands?

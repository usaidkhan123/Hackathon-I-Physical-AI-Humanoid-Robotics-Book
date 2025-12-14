---
id: lesson-04-actions
title: Lesson 4 - ROS 2 Actions
sidebar_position: 4
description: Learn how to use ROS 2 actions for long-running, asynchronous tasks with feedback.
---

# Lesson 4: ROS 2 Actions

## Lesson Objective

By the end of this lesson, you will be able to write a ROS 2 action client and server in Python using `rclpy` and use them to implement a long-running, asynchronous task with feedback.

## Prerequisites

- Completion of Lesson 3: ROS 2 Services

## Concept Explanation

ROS 2 actions are a communication mechanism for long-running, asynchronous tasks with feedback. A node that provides an action is called an "action server," and a node that uses an action is called an "action client." When a client calls an action, it sends a goal to the server. The server then begins executing the goal and can provide feedback to the client as it does so. When the server is finished, it sends a result to the client. This is a great way to implement tasks that take a long time to complete, such as navigating to a goal or manipulating an object.

## Step-by-Step Technical Breakdown

1.  **Define an action type**: We will use a standard ROS 2 action type to define the structure of our goal, result, and feedback messages.
2.  **Write an action server**: We will write a Python node that provides an action.
3.  **Write an action client**: We will write another Python node that calls the action.
4.  **Run the nodes**: We will run both nodes and see the client send a goal to the server, the server execute the goal and provide feedback, and the client receive the result.

## Real-World Analogy

Think of a ROS 2 action as ordering a pizza. The action client is the customer, who places an order for a pizza with specific toppings (the goal). The action server is the pizza shop, which accepts the order and begins making the pizza. As the pizza is being made, the shop might provide updates to the customer, such as "Your pizza is in the oven" (the feedback). When the pizza is ready, the shop delivers it to the customer (the result).

## Hands-On Task

Follow the step-by-step technical breakdown to create an action client and server and use them to implement a long-running, asynchronous task with feedback.

## Python + ROS2 Code Example

**Action Server:**
```python
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from action_tutorials_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]
        for i in range(1, goal_handle.request.order):
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
            self.get_logger().info(f'Feedback: {feedback_msg.partial_sequence}')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result

def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Action Client:**
```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_tutorials_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: {feedback_msg.feedback.partial_sequence}')

def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionClient()
    node.send_goal(10)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

## Common Mistakes & Debugging Tips

- **Forgetting to wait for the action server**: An action client must wait for the action server to be available before sending a goal.
- **Not handling goal rejection**: An action server can reject a goal if it is not able to process it. An action client should always check if the goal was accepted before proceeding.

## Mini Assessment

1.  What are ROS 2 actions used for?
2.  What is the difference between an action client and an action server?
3.  What are the three message types that make up an action type?
4.  How can you see a list of all the active actions on a ROS 2 system?

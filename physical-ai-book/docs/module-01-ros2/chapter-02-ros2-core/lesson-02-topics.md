---
id: lesson-02-topics
title: Lesson 2 - ROS 2 Topics
sidebar_position: 2
description: Learn how to use ROS 2 topics for asynchronous, one-to-many communication between nodes.
---

# Lesson 2: ROS 2 Topics

## Lesson Objective

By the end of this lesson, you will be able to write a ROS 2 publisher and subscriber in Python using `rclpy` and use them to communicate between two nodes.

## Prerequisites

- Completion of Lesson 1: ROS 2 Nodes

## Concept Explanation

ROS 2 topics are a communication mechanism for asynchronous, one-to-many communication between nodes. A node that wants to share information "publishes" messages to a topic, and any number of other nodes can "subscribe" to that topic to receive the messages. This is a great way to stream data, such as sensor readings or robot state information.

## Step-by-Step Technical Breakdown

1.  **Define a message type**: We will use a standard ROS 2 message type to define the structure of our messages.
2.  **Write a publisher**: We will write a Python node that publishes messages to a topic.
3.  **Write a subscriber**: We will write another Python node that subscribes to the topic and receives the messages.
4.  **Run the nodes**: We will run both nodes and see them communicate with each other.

## Real-World Analogy

Think of a ROS 2 topic as a radio station. The publisher is the radio DJ, who broadcasts music and talk shows to anyone who wants to listen. The subscribers are the listeners, who can tune in to the radio station to hear the broadcast. Any number of listeners can tune in to the same station, and they don't need to know anything about the DJ or where the broadcast is coming from.

## Hands-On Task

Follow the step-by-step technical breakdown to create a publisher and subscriber and use them to communicate between two nodes.

## Python + ROS2 Code Example

**Publisher:**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyPublisher(Node):
    def __init__(self):
        super().__init__('my_publisher')
        self.publisher_ = self.create_publisher(String, 'my_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello, World! {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = MyPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Subscriber:**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MySubscriber(Node):
    def __init__(self):
        super().__init__('my_subscriber')
        self.subscription = self.create_subscription(
            String,
            'my_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = MySubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Common Mistakes & Debugging Tips

- **Mismatched message types**: The publisher and subscriber must use the exact same message type. If they don't, you will get an error when you try to run them.
- **Mismatched topic names**: The publisher and subscriber must also use the exact same topic name. If they don't, they won't be able to communicate with each other. You can use `ros2 topic list` to see a list of all the active topics on the system.

## Mini Assessment

1.  What are ROS 2 topics used for?
2.  What is the difference between a publisher and a subscriber?
3.  What is a message type?
4.  How can you see a list of all the active topics on a ROS 2 system?

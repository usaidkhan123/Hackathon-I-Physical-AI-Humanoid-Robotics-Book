---
id: lesson-01-python-to-ros
title: Lesson 1 - Python to ROS
sidebar_position: 1
description: Learn the basics of using rclpy to create a simple ROS 2 node in Python.
---

# Lesson 1: Python to ROS

## Lesson Objective

By the end of this lesson, you will be able to write a basic ROS 2 node in Python using the `rclpy` library, and understand the fundamental structure of a `rclpy` program.

## Prerequisites

- Completion of Chapter 2: ROS 2 Core Concepts

## Concept Explanation

The `rclpy` library provides the necessary tools to interface Python with the ROS 2 communication middleware. It allows you to create nodes, and from there, create publishers, subscribers, and other ROS 2 communication objects. The basic structure of a `rclpy` program involves initializing the library, creating a node, "spinning" the node to allow it to process events and callbacks, and then shutting down the library.

## Step-by-Step Technical Breakdown

1.  **Import `rclpy` and `Node`**: The first step is to import the necessary classes from the `rclpy` library.
2.  **Initialize `rclpy`**: The `rclpy.init()` function initializes the ROS 2 middleware.
3.  **Create a Node**: Create a class that inherits from `rclpy.node.Node` and initialize it with a unique node name.
4.  **Spin the Node**: The `rclpy.spin()` function enters a loop, processing any incoming messages or service calls. This keeps your node alive and responsive.
5.  **Shutdown `rclpy`**: The `rclpy.shutdown()` function cleans up the ROS 2 middleware resources.

## Real-World Analogy

Think of `rclpy` as a translator that allows your Python program to speak the language of ROS 2. `rclpy.init()` is like opening the dictionary, creating a `Node` is like giving your program a name tag so others can identify it, and `rclpy.spin()` is like telling your program to start listening and responding to conversations.

## Hands-On Task

Create a Python file, import `rclpy`, and write the code to initialize `rclpy`, create a simple node that logs a "Hello" message, spin the node, and then shut down. Run this Python script and observe the output.

## Python + ROS2 Code Example

```python
import rclpy
from rclpy.node import Node

class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_python_node')
        self.get_logger().info('Hello from my first rclpy node!')

def main(args=None):
    # 1. Initialize rclpy
    rclpy.init(args=args)

    # 2. Create the Node
    simple_node = SimpleNode()

    # 3. Spin the Node to keep it alive
    rclpy.spin(simple_node)

    # 4. Shutdown rclpy
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Common Mistakes & Debugging Tips

- **Forgetting `rclpy.init()`**: The most common mistake is forgetting to initialize the `rclpy` library. This will result in an error when you try to create a node.
- **Node name collision**: Every running node in a ROS 2 system must have a unique name. If you try to run two nodes with the same name, the second one will fail to start.

## Mini Assessment

1.  What is the purpose of the `rclpy` library?
2.  What does the `rclpy.init()` function do?
3.  What is the purpose of `rclpy.spin()`?
4.  Why is it important for a ROS 2 node to have a unique name?

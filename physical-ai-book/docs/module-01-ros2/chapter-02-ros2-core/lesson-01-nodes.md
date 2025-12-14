---
id: lesson-01-nodes
title: Lesson 1 - ROS 2 Nodes
sidebar_position: 1
description: Learn how to create and manage ROS 2 nodes, the main executable programs in a ROS 2 system.
---

# Lesson 1: ROS 2 Nodes

## Lesson Objective

By the end of this lesson, you will be able to write a simple ROS 2 node in Python using `rclpy` and run it from the command line.

## Prerequisites

- Completion of Chapter 1: Foundations of Physical AI

## Concept Explanation

A ROS 2 node is the main executable program in a ROS 2 system. Each node should be responsible for a single, well-defined task, such as controlling a motor, reading a sensor, or planning a path. Nodes can communicate with each other using ROS 2 communication mechanisms like topics, services, and actions.

## Step-by-Step Technical Breakdown

1.  **Create a Python package**: We will start by creating a new Python package for our ROS 2 code.
2.  **Write the node**: We will write a simple "Hello, World!" node in Python using the `rclpy` library.
3.  **Build the package**: We will use `colcon`, the ROS 2 build tool, to build our package.
4.  **Run the node**: We will run our node from the command line and see the output.

## Real-World Analogy

Think of a ROS 2 system as a team of specialists working together on a project. Each specialist is a node, and each has a specific role or task. For example, one specialist might be responsible for gathering data (a sensor node), another for processing the data (a data processing node), and a third for making decisions based on the data (a control node).

## Hands-On Task

Follow the step-by-step technical breakdown to create, build, and run your own simple ROS 2 node.

## Python + ROS2 Code Example

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('Hello, ROS 2!')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Common Mistakes & Debugging Tips

- **Forgetting to source the setup file**: Before running a ROS 2 node, you must source the `setup.bash` file in your ROS 2 workspace. This sets up the environment variables that ROS 2 needs to find your packages and nodes.
- **`colcon build` errors**: If you encounter errors when building your package, make sure you have declared all your dependencies in your `package.xml` file.

## Mini Assessment

1.  What is a ROS 2 node?
2.  What is the purpose of the `rclpy` library?
3.  What is the command to build a ROS 2 package?
4.  What is the command to run a ROS 2 node?

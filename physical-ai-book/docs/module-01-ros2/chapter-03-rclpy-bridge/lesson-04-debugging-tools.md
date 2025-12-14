---
id: lesson-04-debugging-tools
title: Lesson 4 - Debugging Tools
sidebar_position: 4
description: Learn how to use ROS 2 debugging tools to inspect and debug your rclpy programs.
---

# Lesson 4: Debugging Tools

## Lesson Objective

By the end of this lesson, you will be able to use the `ros2` command-line tools to inspect the topics, services, and actions in a running ROS 2 system, and to publish messages and call services from the command line.

## Prerequisites

- Completion of Lesson 3: Message Types

## Concept Explanation

ROS 2 comes with a powerful set of command-line tools that you can use to inspect and debug your ROS 2 system. These tools allow you to see what topics, services, and actions are currently active, to view the messages being published on a topic, and to call services and actions from the command line.

## Step-by-Step Technical Breakdown

1.  **`ros2 topic list`**: We will use this command to see a list of all the active topics in the system.
2.  **`ros2 topic echo`**: We will use this command to view the messages being published on a specific topic.
3.  **`ros2 topic pub`**: We will use this command to publish a message to a topic from the command line.
4.  **`ros2 service list`**: We will use this command to see a list of all the active services in the system.
5.  **`ros2 service call`**: We will use this command to call a service from the command line.
6.  **`ros2 action list`**: We will use this command to see a list of all the active actions in the system.
7.  **`ros2 action send_goal`**: We will use this command to send a goal to an action server from the command line.

## Real-World Analogy

Think of the ROS 2 debugging tools as a set of diagnostic tools for a car. You can use them to check the oil level, the tire pressure, and the engine temperature. You can also use them to start the engine, rev the engine, and honk the horn.

## Hands-On Task

Run the publisher and subscriber nodes from the previous lessons. Then, use the `ros2` command-line tools to inspect the topic, echo the messages, and publish a message from the command line. Do the same for the service and action nodes.

## Python + ROS2 Code Example

This lesson focuses on command-line tools, so there are no new Python code examples. You will be using the nodes created in previous lessons.

**Example `ros2` commands:**
```bash
# List all topics
ros2 topic list

# Echo messages from a topic
ros2 topic echo /my_topic

# Publish a message to a topic
ros2 topic pub /my_topic std_msgs/msg/String "data: 'Hello from the command line'"

# List all services
ros2 service list

# Call a service
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 10}"
```

## Common Mistakes & Debugging Tips

- **Incorrect message syntax for `ros2 topic pub` or `ros2 service call`**: The message content must be in valid YAML format. If you get a syntax error, double-check your YAML.
- **Forgetting to source the workspace**: As always, ensure your ROS 2 workspace is sourced so the command-line tools can find your custom packages and interfaces.

## Mini Assessment

1.  What command would you use to see a list of all active topics?
2.  How can you view the messages being published on a specific topic?
3.  What command allows you to publish a message from the command line?
4.  How do you call a service from the command line?

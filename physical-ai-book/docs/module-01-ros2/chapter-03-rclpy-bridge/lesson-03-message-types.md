---
id: lesson-03-message-types
title: Lesson 3 - Message Types
sidebar_position: 3
description: Learn about ROS 2 message types and how to use them in your rclpy programs.
---

# Lesson 3: Message Types

## Lesson Objective

By the end of this lesson, you will be able to find and use standard ROS 2 message types, and understand how to create your own custom message types.

## Prerequisites

- Completion of Lesson 2: Robot Controllers

## Concept Explanation

ROS 2 message types are the data structures that are used to send information between nodes. Each topic, service, and action has a message type associated with it. ROS 2 comes with a large number of standard message types for common robotics tasks, such as sending sensor data or controlling motors. You can also create your own custom message types for your specific application.

## Step-by-Step Technical Breakdown

1.  **Find a standard message type**: We will use the `ros2 interface show` command to find a standard message type for sending a string.
2.  **Use the message type in a program**: We will write a simple publisher and subscriber that use the message type we found.
3.  **Create a custom message type**: We will create a new package and define our own custom message type.
4.  **Use the custom message type in a program**: We will write a publisher and subscriber that use our custom message type.

## Real-World Analogy

Think of ROS 2 message types as the different types of envelopes you can use to send a letter. You can use a standard business envelope for a formal letter, a greeting card envelope for a birthday card, or a padded envelope for a fragile item. You can also create your own custom envelopes for special occasions.

## Hands-On Task

Follow the step-by-step technical breakdown to find and use a standard message type, and then create and use your own custom message type.

## Python + ROS2 Code Example

**Using a standard message type (`std_msgs/String`):**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# (See publisher/subscriber examples in Chapter 2, Lesson 2)
```

**Defining a custom message type (`my_package/msg/MyMessage.msg`):**
```
string data
int32 value
```

**Using the custom message type:**
```python
import rclpy
from rclpy.node import Node
from my_package.msg import MyMessage

class MyCustomPublisher(Node):
    def __init__(self):
        super().__init__('my_custom_publisher')
        self.publisher_ = self.create_publisher(MyMessage, 'custom_topic', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = MyMessage()
        msg.data = 'Hello'
        msg.value = 42
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: data="{msg.data}", value={msg.value}')

# (Subscriber would be similar, with a callback that receives a MyMessage object)
```

## Common Mistakes & Debugging Tips

- **Forgetting to build the package with the custom message**: After creating a custom message type, you must build your package with `colcon build` before you can use it in your programs.
- **Incorrectly importing the custom message**: The import statement for a custom message is `from my_package.msg import MyMessage`. Make sure you replace `my_package` and `MyMessage` with the actual names of your package and message.

## Mini Assessment

1.  What are ROS 2 message types?
2.  How can you find a standard message type?
3.  How do you create a custom message type?
4.  What must you do after creating a custom message type before you can use it in your programs?

---
id: lesson-02-robot-controllers
title: Lesson 2 - Robot Controllers
sidebar_position: 2
description: Learn how to write a simple robot controller in Python using rclpy.
---

# Lesson 2: Robot Controllers

## Lesson Objective

By the end of this lesson, you will be able to write a simple robot controller node in Python that subscribes to sensor data and publishes motor commands.

## Prerequisites

- Completion of Lesson 1: Python to ROS

## Concept Explanation

A robot controller is a ROS 2 node that takes in sensor data, processes it, and then publishes commands to the robot's actuators (e.g., motors). This is the core of most robotics applications, as it is what allows the robot to react to its environment and perform tasks.

## Step-by-Step Technical Breakdown

1.  **Create a Controller Node**: We will create a new Python class for our controller node.
2.  **Create a Subscriber**: The controller will subscribe to a topic that provides sensor data (e.g., from a laser scanner).
3.  **Create a Publisher**: The controller will publish messages to a topic that controls the robot's motors (e.g., a `Twist` message for a differential drive robot).
4.  **Implement Control Logic**: We will write the logic that takes the sensor data and decides what motor commands to publish.

## Real-World Analogy

Think of a robot controller as the "brain" of the robot. It receives information from the robot's "senses" (the sensors) and then tells the robot's "muscles" (the motors) what to do.

## Hands-On Task

Write a simple controller that subscribes to a `/scan` topic (of type `sensor_msgs/LaserScan`) and publishes to a `/cmd_vel` topic (of type `geometry_msgs/Twist`). The controller should make the robot move forward until it gets too close to an obstacle, at which point it should stop.

## Python + ROS2 Code Example

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription  # prevent unused variable warning

    def scan_callback(self, msg):
        twist = Twist()
        # Simple obstacle avoidance: if the minimum range is less than 1 meter, stop.
        if min(msg.ranges) < 1.0:
            twist.linear.x = 0.0
        else:
            twist.linear.x = 0.5
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Common Mistakes & Debugging Tips

- **Incorrect topic names or types**: Make sure that the topic names and message types in your controller match the ones being used by your robot's sensors and actuators. You can use `ros2 topic info <topic_name>` to check the message type of a topic.
- **Bugs in control logic**: The control logic is where most bugs in a robot controller are found. Use `self.get_logger().info()` statements to print out the values of your variables and make sure that your logic is behaving as expected.

## Mini Assessment

1.  What is a robot controller?
2.  What is the role of a subscriber in a robot controller?
3.  What is the role of a publisher in a robot controller?
4.  What is a common type of message used to control a robot's motors?

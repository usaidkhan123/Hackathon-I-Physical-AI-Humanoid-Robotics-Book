---
id: lesson-04-ros2-gazebo-interface
title: "Lesson 4: ROS 2 and Gazebo Interface"
sidebar_position: 4
description: Learn how to connect ROS 2 with Gazebo to control your simulated robots.
---

## Lesson Objective

By the end of this lesson, you will be able to launch a Gazebo simulation from a ROS 2 launch file and use ROS 2 topics to control a robot and read sensor data.

## Prerequisites

- Completion of the previous lessons in this chapter.
- Basic understanding of ROS 2 concepts (nodes, topics, launch files).

## Concept Explanation

The true power of Gazebo is realized when it is integrated with ROS 2. The `gazebo_ros_pkgs` provide a set of ROS 2 packages that allow seamless communication between Gazebo and the ROS 2 ecosystem.

This is achieved through a special Gazebo plugin called `libgazebo_ros_factory.so` and other related plugins that expose Gazebo's functionality through ROS 2 interfaces:
- **Topics:** Robot commands (like velocity) are sent to Gazebo on topics. Sensor data (like laser scans) are published by Gazebo on topics.
- **Services:** You can use ROS 2 services to do things like spawn or delete models in the simulation.
- **Launch Files:** ROS 2 launch files are used to start and configure both the Gazebo simulation and the ROS 2 nodes that interact with it.

## Step-by-Step Technical Breakdown

### 1. Install `gazebo_ros_pkgs`

First, you need to install the packages that bridge ROS 2 and Gazebo.
```bash
sudo apt-get install ros-humble-gazebo-ros-pkgs
```

### 2. Create a Simple Launch File

Create a new ROS 2 package if you don't have one. Inside the `launch` directory, create a file named `gazebo_world.launch.py`.

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Find the path to the world file
    world_file_name = 'my_world.world'
    world_path = os.path.join(os.getcwd(), 'src', 'your_package_name', 'worlds', world_file_name) # Adjust to your package structure

    return LaunchDescription([
        # Launch Gazebo with your world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
    ])
```
*Note: You'll need to create a `worlds` directory in your package and move `my_world.world` into it.*

### 3. Control a Robot with ROS 2

Let's use the `pioneer2dx` robot from the previous lesson. Its Gazebo plugin exposes a `/cmd_vel` topic for receiving velocity commands.

Open a new terminal and run the following command to publish a velocity command:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
```
The robot in your Gazebo simulation should start moving in a curve.

### 4. Read Sensor Data

The `pioneer2dx` also has a simulated laser scanner that publishes data on the `/laser_scan` topic.

In a new terminal, run:
```bash
ros2 topic echo /laser_scan
```
You will see a stream of `sensor_msgs/msg/LaserScan` messages representing the distance readings from the simulated scanner.

## Real-World Analogy

Connecting ROS 2 to Gazebo is like giving your robot a brain and nervous system. Gazebo is the physical body, living in its simulated world. ROS 2 is the mind that processes information from the body's senses (Gazebo sensors) and sends commands to its muscles (Gazebo actuators/joints).

## Hands-On Task

1. Create a ROS 2 launch file that starts Gazebo with the `pioneer2dx.world`.
2. Use `ros2 topic pub` to make the robot drive forward in a straight line.
3. Use `ros2 topic echo` to view the `/odom` topic, which provides information about the robot's estimated position and velocity.

## Common Mistakes & Debugging Tips

- **Topics Not Appearing:** If you can't see the expected ROS 2 topics, make sure the `gazebo_ros_pkgs` are installed and that you included the necessary Gazebo plugins in your world or URDF file. Use `ros2 topic list` to see all available topics.
- **`ros2` command not found:** Ensure you have sourced your ROS 2 installation. `source /opt/ros/humble/setup.bash`.

## Mini Assessment

1. What package provides the bridge between ROS 2 and Gazebo?
2. How can you start Gazebo from a ROS 2 launch file?
3. What type of ROS message is typically used to send velocity commands?
4. True or False: You need to write a custom plugin to read LiDAR data in ROS 2.
5. What does the `/odom` topic typically represent?

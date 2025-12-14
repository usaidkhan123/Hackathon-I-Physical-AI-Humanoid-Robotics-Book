---
id: lesson-04-testing-models
title: Lesson 4 - Testing Models
sidebar_position: 4
description: Learn how to view and test your URDF model in RViz.
---

# Lesson 4: Testing Models

## Lesson Objective

By the end of this lesson, you will be able to launch a ROS 2 node that publishes the state of your URDF model and view the model in RViz.

## Prerequisites

- Completion of Lesson 3: Sensors in URDF

## Concept Explanation

RViz is a powerful 3D visualization tool for ROS 2. It can be used to visualize the state of your robot, the data from its sensors, and the output of its algorithms. To view a URDF model in RViz, you need to have a ROS 2 node that publishes the state of the model as a set of TF2 transforms. The `robot_state_publisher` package provides such a node.

## Step-by-Step Technical Breakdown

1.  **Create a launch file**: We will create a ROS 2 launch file to start the `robot_state_publisher` node and RViz.
2.  **Configure the `robot_state_publisher`**: We will configure the `robot_state_publisher` to load our URDF model.
3.  **Configure RViz**: We will configure RViz to display the robot model.
4.  **Launch the file**: We will launch the file and see our robot model in RViz.

## Real-World Analogy

Think of RViz as a virtual "test track" for your robot. You can use it to see how your robot model looks, how its joints move, and how its sensors perceive the world, all without needing to have a physical robot.

## Hands-On Task

Follow the step-by-step technical breakdown to create a launch file that starts the `robot_state_publisher` and RViz, and then view your robot model in RViz.

## Python + ROS2 Code Example

**urdf_test.launch.py:**
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'two_link_arm.urdf'

    urdf = os.path.join(
        get_package_share_directory('your_package_name'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory('your_package_name'), 'config.rviz')])
    ])
```

## Common Mistakes & Debugging Tips

- **Incorrect package name**: Make sure you replace `your_package_name` with the actual name of your package.
- **RViz configuration**: You will need to add a "RobotModel" display to RViz to see your robot model. You may also need to set the "Fixed Frame" to the base link of your robot.

## Mini Assessment

1.  What is RViz?
2.  What is the role of the `robot_state_publisher`?
3.  What is a ROS 2 launch file?
4.  What do you need to do in RViz to see your robot model?

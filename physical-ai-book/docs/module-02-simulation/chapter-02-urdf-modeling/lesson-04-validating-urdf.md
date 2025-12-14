---
id: lesson-04-validating-urdf
title: "Lesson 4: Validating and Debugging URDF"
sidebar_position: 4
description: Learn tools and techniques to ensure your URDF is correct.
---

## Lesson Objective

By the end of this lesson, you will be able to use command-line tools and graphical visualizations to validate your URDF files, identify problems, and debug your robot models.

## Prerequisites

- Completion of the previous lessons in this chapter.
- A URDF or XACRO file to test.

## Concept Explanation

A small syntax error in a URDF file can lead to frustrating and hard-to-diagnose problems in the simulation. Fortunately, ROS 2 provides tools to help you catch these errors early.

The main tools for URDF validation are:
- **`check_urdf`:** A simple command-line tool that parses a URDF file and reports any syntax errors. It's a quick first-pass check.
- **`urdf_to_graphiz`:** This tool creates a PDF visual of your robot's link and joint tree, which is invaluable for understanding the structure of a complex model.
- **RViz:** The ROS 2 visualizer, RViz, can display a 3D model of your robot from a URDF. This is the best way to check that your visuals and joint origins are set up correctly without the overhead of a full physics simulation.

## Step-by-Step Technical Breakdown

### 1. Checking the URDF Syntax

Let's start with a deliberately broken URDF. Create a file `broken.urdf` with a missing closing tag:
```xml
<?xml version="1.0"?>
<robot name="broken_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    <!-- Missing </visual> tag -->
  </link>
</robot>
```
Now, run `check_urdf` on it:
```bash
check_urdf broken.urdf
```
The tool will fail and print an error message indicating an XML parsing error, helping you to quickly identify the mistake.

### 2. Visualizing the Robot Tree

For a correct URDF, like the one for your `simple_arm` from a previous lesson, you can generate a graph of its structure.
First, make sure `graphviz` is installed:
```bash
sudo apt-get install graphviz
```
Then, run the tool:
```bash
urdf_to_graphiz simple_arm.urdf
```
This will generate a `simple_arm.pdf` file. Open it to see a diagram showing `base_link` connected to `arm_link` via `base_to_arm_joint`.

### 3. Visualizing the Model in RViz

RViz is the most powerful tool for debugging URDFs. It requires a few more steps to get working. You need a small ROS 2 node to publish the robot's state.

Create a launch file, `display.launch.py`:
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    urdf_file_name = 'simple_arm.urdf.xacro'
    urdf_path = os.path.join(
        get_package_share_directory('your_package_name'),
        'urdf',
        urdf_file_name)

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', urdf_path])}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('your_package_name'), 'rviz', 'urdf.rviz')]
        )
    ])
```
This launch file does three things:
1.  **`robot_state_publisher`**: Reads the URDF, finds all the non-fixed joints, and creates a `/robot_description` topic.
2.  **`joint_state_publisher_gui`**: Creates a small GUI with sliders to control the joint angles. It publishes the angles on the `/joint_states` topic.
3.  **`rviz2`**: Starts RViz. You will need to add a "RobotModel" display in RViz and set its topic to `/robot_description`.

## Real-World Analogy

Validating a URDF is like proofreading a document.
- `check_urdf` is like a spell-checker, catching basic syntax mistakes.
- `urdf_to_graphiz` is like creating an outline or table of contents, showing you the overall structure.
- RViz is like a 3D preview, letting you see exactly how the final product will look before you "print" it (i.e., send it to the physics simulator).

## Hands-On Task

1.  Intentionally introduce an error into your `simple_arm.urdf.xacro` file (e.g., misspell a tag). Run the `xacro` command and see the error. Fix it.
2.  Generate the PDF graph for your `simple_arm` and verify its structure.
3.  Create the `display.launch.py` file shown above (adjusting paths for your package).
4.  Create a simple RViz configuration file (`urdf.rviz`) that includes the RobotModel display.
5.  Run `ros2 launch your_package_name display.launch.py`.
6.  In the `joint_state_publisher_gui` window, move the slider for the `base_to_arm_joint`. You should see the arm move in RViz.

## Common Mistakes & Debugging Tips

- **RViz shows "No transform" errors:** This is a classic problem. It means the `robot_state_publisher` is not running correctly or RViz can't get the TF (transform) data. Make sure all nodes in your launch file started correctly. The `fixed_frame` in RViz should typically be set to your robot's base link (e.g., `base_link`).
- **`xacro` command fails in launch file:** Make sure the `xacro` executable is installed and in your system's PATH. The `Command(['xacro ', urdf_path])` syntax is a robust way to handle this.

## Mini Assessment

1.  What is the quickest way to check a URDF file for XML errors?
2.  What does the `urdf_to_graphiz` tool produce?
3.  What is the role of `robot_state_publisher`?
4.  What is the role of `joint_state_publisher_gui`?
5.  In RViz, what "Display" do you add to see your robot model?

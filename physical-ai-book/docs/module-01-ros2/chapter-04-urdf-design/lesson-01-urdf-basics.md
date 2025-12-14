---
id: lesson-01-urdf-basics
title: Lesson 1 - URDF Basics
sidebar_position: 1
description: Learn the basic syntax of the URDF format and how to create a simple URDF file.
---

# Lesson 1: URDF Basics

## Lesson Objective

By the end of this lesson, you will be able to understand the basic syntax of the URDF format and create a simple URDF file that defines a single link.

## Prerequisites

- Completion of Chapter 3: The rclpy Bridge

## Concept Explanation

URDF is an XML-based format for representing a robot model. The core components of a URDF file are `<link>` and `<joint>` elements. A `<link>` element describes a rigid part of the robot, including its visual geometry, collision geometry, and inertial properties. A `<joint>` element describes how two links are connected to each other.

## Step-by-Step Technical Breakdown

1.  **Create a URDF file**: We will create a new file with a `.urdf` extension.
2.  **Add the `<robot>` element**: The root element of a URDF file is the `<robot>` element.
3.  **Add a `<link>` element**: We will add a `<link>` element to define a single link.
4.  **Add `<visual>`, `<collision>`, and `<inertial>` elements**: We will add these elements to the `<link>` element to define its visual geometry, collision geometry, and inertial properties.

## Real-World Analogy

Think of a URDF file as a set of blueprints for a robot. The `<link>` elements are the individual parts of the robot, and the `<joint>` elements are the instructions for how to connect the parts together.

## Hands-On Task

Follow the step-by-step technical breakdown to create a simple URDF file that defines a single, box-shaped link.

## Python + ROS2 Code Example

**simple_robot.urdf:**
```xml
<?xml version="1.0"?>
<robot name="simple_robot">

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.5" />
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.5" />
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0" />
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.04" />
    </inertial>
  </link>

</robot>
```
*Note: This is an XML file, not a Python/ROS 2 script. No `rclpy` code is needed for this lesson.*

## Common Mistakes & Debugging Tips

- **XML syntax errors**: URDF files are XML files, so they must follow XML syntax rules. Make sure you close all your tags and that your file is well-formed.
- **Forgetting required elements**: The `<robot>` element and at least one `<link>` element are required in a URDF file.

## Mini Assessment

1.  What is URDF?
2.  What is the root element of a URDF file?
3.  What is a `<link>` element used for?
4.  What are the three main elements that can be defined within a `<link>` element?

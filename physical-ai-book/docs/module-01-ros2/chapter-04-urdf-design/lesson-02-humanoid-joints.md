---
id: lesson-02-humanoid-joints
title: Lesson 2 - Humanoid Joints
sidebar_position: 2
description: Learn how to use joints to connect links together and create a simple humanoid robot model.
---

# Lesson 2: Humanoid Joints

## Lesson Objective

By the end of this lesson, you will be able to use `<joint>` elements to connect `<link>` elements together and create a simple, articulated robot model.

## Prerequisites

- Completion of Lesson 1: URDF Basics

## Concept Explanation

A `<joint>` element describes the kinematic and dynamic properties of a joint between two links. The most important properties of a joint are its type (e.g., `revolute`, `continuous`, `prismatic`, `fixed`), its parent and child links, and its origin (i.e., its position and orientation relative to the parent link).

## Step-by-Step Technical Breakdown

1.  **Add a second link**: We will add a second link to our URDF file.
2.  **Add a `<joint>` element**: We will add a `<joint>` element to connect the two links together.
3.  **Set the joint type**: We will set the joint type to `revolute` to create a hinge joint.
4.  **Set the parent and child links**: We will set the `parent` and `child` attributes of the joint to specify which links it connects.
5.  **Set the joint origin**: We will set the `origin` of the joint to specify its position and orientation relative to the parent link.

## Real-World Analogy

Think of the joints in your own body. Your elbow is a `revolute` joint that allows your forearm to rotate relative to your upper arm. Your knee is also a `revolute` joint. Your shoulder is a more complex, ball-and-socket joint. In URDF, we can model these different types of joints to create a realistic robot model.

## Hands-On Task

Follow the step-by-step technical breakdown to add a second link and a `revolute` joint to your URDF file to create a simple, two-link arm.

## Python + ROS2 Code Example

**two_link_arm.urdf:**
```xml
<?xml version="1.0"?>
<robot name="two_link_arm">

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.5" />
      </geometry>
    </visual>
  </link>

  <link name="link_1">
    <visual>
      <geometry>
        <box size="0.5 0.1 0.1" />
      </geometry>
    </visual>
  </link>

  <joint name="joint_1" type="revolute">
    <parent link="base_link" />
    <child link="link_1" />
    <origin xyz="0.25 0 0" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1" />
  </joint>

</robot>
```
*Note: This is an XML file. No `rclpy` code is needed for this lesson.*

## Common Mistakes & Debugging Tips

- **Incorrect joint origin**: The `origin` of the joint is specified relative to the parent link's origin. It's easy to get this wrong, so it's a good idea to use a visualization tool like RViz to check that your joints are in the correct position.
- **Incorrect joint axis**: The `axis` of the joint specifies the axis of rotation for a `revolute` joint or the axis of translation for a `prismatic` joint.

## Mini Assessment

1.  What is a `<joint>` element used for?
2.  What are some of the different types of joints that can be used in a URDF file?
3.  What is the purpose of the `origin` element within a `<joint>` element?
4.  What does the `axis` element specify for a `revolute` joint?

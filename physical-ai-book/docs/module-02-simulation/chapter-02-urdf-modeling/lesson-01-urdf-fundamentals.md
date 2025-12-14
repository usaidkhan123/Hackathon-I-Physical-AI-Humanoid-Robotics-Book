---
id: lesson-01-urdf-fundamentals
title: "Lesson 1: URDF Fundamentals"
sidebar_position: 1
description: Learn the basic building blocks of a URDF file.
---

## Lesson Objective

By the end of this lesson, you will be able to write a simple URDF file that defines a single link and understand the core concepts of links and joints.

## Prerequisites

- Basic understanding of XML.
- Gazebo installed and working.

## Concept Explanation

A URDF file describes a robot as a tree of **links** connected by **joints**.

- **`<link>`:** A rigid body with physical properties like mass, inertia, and visual/collision geometries. Think of it as a bone or a solid part of the robot.
- **`<joint>`:** Defines the kinematic and dynamic properties of the connection between two links. It specifies the type of motion allowed (e.g., revolute, prismatic) and the axis of motion.

The entire robot structure is enclosed within a single `<robot>` tag, which has a `name` attribute.

## Step-by-Step Technical Breakdown

### 1. Create a Basic URDF File

Create a file named `single_link.urdf` and add the following content:

```xml
<?xml version="1.0"?>
<robot name="my_first_robot">

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <material name="blue">
        <color rgba="0.0 0.2 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

</robot>
```

- **`<robot name="...">`**: The root element of any URDF.
- **`<link name="...">`**: Defines a new link. The `base_link` is conventionally the root link of the robot.
- **`<visual>`**: Describes how the link looks. It contains a `<geometry>` tag (e.g., `<box>`, `<cylinder>`, `<sphere>`, or `<mesh>`).
- **`<collision>`**: Defines the collision model. It's often similar to the visual geometry but can be simplified for performance.
- **`<inertial>`**: Defines the dynamic properties: `<mass>` and `<inertia>` tensor.

### 2. Spawning the URDF in Gazebo

To see your robot, you need to spawn it into a running Gazebo instance.

First, launch an empty Gazebo world:
```bash
gazebo --verbose
```

In a separate terminal, use the `ros2 run` command to call the `spawn_entity.py` script, which is part of `gazebo_ros`:
```bash
ros2 run gazebo_ros spawn_entity.py -entity my_first_robot -file single_link.urdf
```
You should see your blue box appear in the Gazebo window.

## Real-World Analogy

Think of building a robot with LEGOs. Each LEGO brick is a `<link>`. The little plastic studs that connect the bricks are the `<joint>`s. The URDF file is the instruction manual that tells you which brick to connect where.

## Hands-On Task

Create a new URDF file for a robot named `simple_arm`. This robot should have two links:
1.  A `base_link` (a small box).
2.  An `arm_link` (a long, thin cylinder).

Connect them with a `revolute` joint (a hinge) named `base_to_arm_joint`. For now, just define the links and the joint connecting them. We'll make it move in a later lesson.

Here's a hint for the joint:
```xml
<joint name="base_to_arm_joint" type="revolute">
  <parent link="base_link"/>
  <child link="arm_link"/>
  <axis xyz="0 1 0"/>
  <limit effort="10" velocity="1.0"/>
</joint>
```

## Common Mistakes & Debugging Tips

- **Missing Inertial Tag:** While Gazebo might still load a model without an `<inertial>` tag, the physics simulation will be incorrect. Always define mass and inertia for every link.
- **`spawn_entity` Fails:** This usually means Gazebo isn't running or there's an error in your URDF file. Check the terminal output from Gazebo for XML parsing errors.

## Mini Assessment

1. What are the two fundamental components of a URDF robot model?
2. What tag defines the physical properties like mass?
3. What is the purpose of the `<collision>` tag?
4. True or False: The `<visual>` and `<collision>` geometries must be identical.
5. What ROS 2 command is used to load a URDF model into Gazebo?

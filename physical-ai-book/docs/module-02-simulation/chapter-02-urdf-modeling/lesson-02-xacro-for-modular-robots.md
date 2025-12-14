---
id: lesson-02-xacro-for-modular-robots
title: "Lesson 2: XACRO for Modular Robots"
sidebar_position: 2
description: Use the XACRO macro language to make your URDFs cleaner and more reusable.
---

## Lesson Objective

By the end of this lesson, you will be able to convert a standard URDF file into a XACRO file, define and use macros, and create more modular and readable robot descriptions.

## Prerequisites

- Completion of Lesson 1: URDF Fundamentals.
- Familiarity with the basic structure of a URDF file.

## Concept Explanation

**XACRO (XML Macros)** is a pre-processor that extends the functionality of URDF. As robots get more complex, URDF files can become very long, repetitive, and difficult to manage. XACRO solves this by introducing:
- **Constants:** Define values (like `pi` or a specific link length) and reuse them throughout the file.
- **Macros:** Create reusable blocks of XML, similar to functions in a programming language. You can parameterize these macros to create similar but slightly different components.
- **Includes:** Split your robot description across multiple files for better organization.

The standard workflow is to write a `.urdf.xacro` file and then use the `xacro` tool to convert it into a final `.urdf` file that Gazebo and ROS 2 can understand.

## Step-by-Step Technical Breakdown

### 1. Converting a URDF to XACRO

Take the `single_link.urdf` from the previous lesson and rename it to `single_link.urdf.xacro`.

Add the `xmlns:xacro` namespace to the `<robot>` tag. This is crucial for the XACRO parser to work.

```xml
<?xml version="1.0"?>
<robot name="my_first_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  ...
</robot>
```

### 2. Using Properties (Constants)

Let's define the box dimensions as properties. This makes it easy to change the size later.

```xml
<xacro:property name="box_x" value="0.6" />
<xacro:property name="box_y" value="0.4" />
<xacro:property name="box_z" value="0.2" />

<link name="base_link">
  <visual>
    <geometry>
      <box size="${box_x} ${box_y} ${box_z}"/>
    </geometry>
    ...
  </visual>
  <collision>
    <geometry>
      <box size="${box_x} ${box_y} ${box_z}"/>
    </geometry>
    ...
  </collision>
  ...
</link>
```
We use `${property_name}` to access the value of a property.

### 3. Creating a Macro

Let's create a macro for a generic box link. This is extremely useful for multi-link robots where many components are similar.

```xml
<xacro:macro name="box_link" params="name mass *origin">
  <link name="${name}">
    <inertial>
      <mass value="${mass}"/>
      <xacro:insert_block name="origin" />
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
  </link>
</xacro:macro>

<!-- Now, use the macro -->
<xacro:box_link name="my_box" mass="1.0">
  <origin xyz="0 0 0.5" rpy="0 0 0"/>
</xacro:box_link>
```
- `<xacro:macro name="..." params="...">`: Defines a macro with a name and a list of parameters.
- `*origin`: This is a special syntax to insert an entire XML block (`<origin>...</origin>`) as a parameter.
- `<xacro:insert_block name="origin" />`: Where the block parameter will be inserted.
- `<xacro:box_link ...>`: This is how you call/invoke the macro.

### 4. Generating the URDF

To see the final output, run the `xacro` tool:
```bash
ros2 run xacro xacro single_link.urdf.xacro > my_robot.urdf
```
Open `my_robot.urdf` to see the generated XML with all properties and macros expanded.

## Real-World Analogy

Using XACRO is like creating a template or a blueprint. Instead of redrawing the same window 20 times for a building plan, you draw one "window" template and just specify its location and size for each instance. This saves time and ensures all windows are consistent.

## Hands-On Task

Take the `simple_arm` URDF you started in the previous lesson's task. Convert it to a `.urdf.xacro` file.
1.  Create properties for the dimensions of the base and the arm links.
2.  Create a macro named `cylinder_link` that takes `name`, `mass`, `radius`, and `length` as parameters.
3.  Use this macro to generate the `arm_link`.
4.  Generate the final URDF using `ros2 run xacro xacro your_file.urdf.xacro`.

## Common Mistakes & Debugging Tips

- **Forgetting the `xmlns:xacro` namespace:** This is the most common error. Without it, the parser won't recognize any of the `<xacro:...>` tags.
- **Syntax Errors:** XACRO is picky. A missing `$` or `{}` can lead to parsing errors. The error messages from `xacro` are usually helpful in pinpointing the problem.
- **Debugging XACRO:** It's often useful to generate the final URDF and inspect it to make sure the macros are expanding as you expect.

## Mini Assessment

1. What is the main purpose of XACRO?
2. What tag is used to define a reusable constant?
3. What is the syntax for using a property in XACRO?
4. True or False: You can load a `.urdf.xacro` file directly into Gazebo.
5. What is a "block" parameter in a XACRO macro?

---
id: lesson-02-building-a-world
title: "Lesson 2: Building a World"
sidebar_position: 2
description: Learn how to create your own custom simulation environments in Gazebo.
---

## Lesson Objective

By the end of this lesson, you will be able to create a simple Gazebo world file containing basic shapes, lighting, and physics properties.

## Prerequisites

- Completion of Lesson 1: Introduction to Gazebo.
- Basic understanding of XML syntax.

## Concept Explanation

Gazebo worlds are defined in `.world` files, which are written in a format called **SDF (Simulation Description Format)**. SDF is an XML-based format that allows you to describe everything in a simulation, from robots and sensors to lighting and environmental physics.

A basic world file includes:
- **`<scene>`:** Defines the ambient lighting and shadows.
- **`<physics>`:** Sets the global physics parameters like gravity.
- **`<include>`:** Allows you to insert pre-made models into your world.
- **`<model>`:** Defines a new model, which can be a simple shape or a complex robot.

## Step-by-Step Technical Breakdown

### 1. Create a World File

Create a new file named `my_world.world` and open it in a text editor.

### 2. Add the SDF Boilerplate

Start with the basic SDF structure:
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="default">
    <!-- Scene and lighting -->
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- Physics Engine -->
    <physics type="ode">
      <gravity>0 0 -9.81</gravity>
    </physics>

    <!-- A ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Your models will go here -->

  </world>
</sdf>
```

### 3. Add a Simple Shape

Let's add a blue box to our world. Add the following inside the `<world>` tags, after the `<include>` for the ground plane.

```xml
<model name="blue_box">
  <pose>1 0 0.5 0 0 0</pose>
  <link name="link">
    <visual name="visual">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
      <material>
        <script>
          <uri>file://media/materials/scripts/gazebo.material</uri>
          <name>Gazebo/Blue</name>
        </script>
      </material>
    </visual>
    <collision name="collision">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
    </collision>
  </link>
</model>
```
- **`<pose>`:** Sets the position (x, y, z) and orientation (roll, pitch, yaw) of the model.
- **`<link>`:** The physical body of the model.
- **`<visual>`:** Defines how the model looks.
- **`<collision>`:** Defines the collision geometry, which is used by the physics engine.

### 4. Launch Your World

Save the file and launch it in Gazebo:
```bash
gazebo my_world.world
```
You should see your blue box sitting on the ground plane.

## Real-World Analogy

Creating a `.world` file is like being a director and set designer for a movie. You decide where to place the lights (`<scene>`), what the laws of physics are (`<physics>`), and where every prop (`<model>`) should go.

## Hands-On Task

Modify your `my_world.world` file to add a second object: a green sphere. Position it at `x=-2`, `y=1`, `z=0.5`. You will need to add a new `<model>` section for the sphere. Use `<sphere><radius>0.5</radius></sphere>` for the geometry.

## Common Mistakes & Debugging Tips

- **XML Errors:** SDF is strict. A missing closing tag can cause the file to fail loading. Gazebo will usually print an error message to the terminal pointing to the problematic line.
- **Model Spawning Inside Others:** If you place two models at the same location, the physics engine might cause them to fly apart violently. Ensure your models have unique, non-overlapping poses.

## Mini Assessment

1. What format are Gazebo world files written in?
2. What tag is used to define the look of a model?
3. What tag is used for the physics properties?
4. How do you set the force of gravity in a world?
5. What happens if a model has a `<visual>` but no `<collision>` tag?

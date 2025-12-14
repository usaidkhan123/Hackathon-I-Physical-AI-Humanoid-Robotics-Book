---
id: lesson-03-models-and-plugins
title: "Lesson 3: Models and Plugins"
sidebar_position: 3
description: Explore Gazebo's model database and learn how to add dynamic behavior with plugins.
---

## Lesson Objective

By the end of this lesson, you will be able to add pre-existing models from Gazebo's database to your world and understand how plugins can be used to add functionality.

## Prerequisites

- Completion of Lesson 2: Building a World.

## Concept Explanation

### Gazebo Models

Gazebo comes with a rich online database of models, including robots, furniture, and other objects. You can easily insert these into your simulation. This saves you from having to model every object from scratch. These models are referenced by a URI (Uniform Resource Identifier), like `model://model_name`.

### Gazebo Plugins

Plugins are shared libraries that can be loaded at runtime to control a simulation. They allow you to add complex behaviors that are not possible with SDF alone, such as:
- Simulating a sensor with noise.
- Creating a robot controller.
- Animating objects in the world.

Plugins can be attached to the world, a model, a sensor, or the system itself.

## Step-by-Step Technical Breakdown

### 1. Finding Models

You can browse the Gazebo model database online or directly through the Gazebo client. In the client, look for the "Insert" tab in the top left panel. It contains a list of models available on your local system and from the online database.

### 2. Inserting a Model into a `.world` File

Let's add a simple robot to our world. Open `my_world.world` and add the following `<include>` tag inside the `<world>` section:
```xml
<!-- A simple differential drive robot -->
<include>
  <uri>model://pioneer2dx</uri>
  <pose>-2 0 0 0 0 0</pose>
</include>
```
This will download the `pioneer2dx` model (if you don't have it locally) and place it in your world at the specified pose.

### 3. Using a Plugin

Let's add a plugin that makes our blue box from the previous lesson move. Modify the `<model name="blue_box">` section by adding the following `<plugin>` tag at the end, just before the closing `</model>` tag:
```xml
<plugin name="move_box" filename="libvelocity_plugin.so">
  <linear_velocity>0.5 0 0</linear_velocity>
  <angular_velocity>0 0 0.2</angular_velocity>
</plugin>
```
This plugin, `libvelocity_plugin.so`, is a simple built-in plugin that applies a constant velocity to a model.

### 4. Relaunch the World

Save your `my_world.world` file and launch it:
```bash
gazebo my_world.world
```
You should now see the Pioneer2dx robot and your blue box, which should be slowly moving and rotating.

## Real-World Analogy

The Gazebo model database is like a giant library of 3D assets. You can check out any model you need for your simulation. Plugins are like adding custom software to your assets. For example, you could add a "self-driving" plugin to a car model to make it navigate a city autonomously.

## Hands-On Task

1. Open Gazebo in an empty world by just running `gazebo`.
2. Use the "Insert" tab to find and add a "Cafeteria" model to the world.
3. Find a "Post Box" model and place it on one of the tables in the cafeteria.
4. Save the world from the `File` menu as `my_cafe.world`.
5. Open `my_cafe.world` in a text editor to see how Gazebo saved the models.

## Common Mistakes & Debugging Tips

- **Model Not Found:** If Gazebo can't find a model, it may be due to a typo in the URI or a network issue preventing it from accessing the online database. Check your spelling and internet connection.
- **Plugin Not Working:** Ensure the plugin's `.so` file is in a path where Gazebo can find it. For built-in plugins, this is usually not an issue. For custom plugins, you may need to set environment variables like `GAZEBO_PLUGIN_PATH`.

## Mini Assessment

1. What is the URI for the Gazebo model database?
2. How can you add a model from the database to your world file?
3. What is the purpose of a Gazebo plugin?
4. Can a plugin be attached to a sensor?
5. What does the `libvelocity_plugin.so` do?

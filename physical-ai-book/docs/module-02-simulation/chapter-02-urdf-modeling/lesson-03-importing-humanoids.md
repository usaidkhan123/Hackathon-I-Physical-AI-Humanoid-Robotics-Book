---
id: lesson-03-importing-humanoids
title: "Lesson 3: Importing Humanoid Models"
sidebar_position: 3
description: Learn how to import complex humanoid robot models into Gazebo using URDF.
---

## Lesson Objective

By the end of this lesson, you will be able to find, download, and spawn a complex humanoid robot model, such as the Valkyrie robot from NASA, into a Gazebo simulation.

## Prerequisites

- Completion of the previous lessons in this chapter.
- `git` installed on your system.

## Concept Explanation

While creating simple robots from scratch is a great learning exercise, many robotics platforms have pre-existing, high-fidelity models available. These are often complex, with dozens or even hundreds of links and joints. Leveraging these existing models is a key skill for any roboticist.

These models are typically distributed as a ROS package containing:
- **URDF/XACRO files:** The core description of the robot's structure.
- **Mesh files:** 3D model files (often in `.dae` or `.stl` format) that define the visual appearance and collision geometry of each link. These are referenced from the URDF.
- **Launch files:** ROS 2 launch files to easily bring up the robot in a simulation.
- **Configuration files:** YAML files that may contain joint limits, controller settings, etc.

## Step-by-Step Technical Breakdown

### 1. Find a Robot Description Package

Let's use NASA's Valkyrie robot as an example. A common way to find these packages is by searching on GitHub or in ROS Index. For Valkyrie, a community-maintained description package is available.

### 2. Clone the Repository

We need to clone the robot description package into our ROS 2 workspace.
```bash
# Navigate to your ROS 2 workspace's src directory
cd ~/ros2_ws/src

# Clone the Valkyrie description package
git clone https://github.com/RoboCup-Humanoid-TC/valkyrie_description.git
```

### 3. Install Dependencies and Build

The package `package.xml` file lists its dependencies. `rosdep` is a tool to install them.
```bash
cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src
colcon build
```

### 4. Spawning the Humanoid in Gazebo

The package will typically provide a launch file to view the robot in RViz or spawn it in Gazebo. Let's find a launch file for Gazebo.

```bash
# First, source the workspace
source ~/ros2_ws/install/setup.bash

# Look for relevant launch files
find ~/ros2_ws/install/valkyrie_description -name "*.launch.py"
```

Let's assume we find a launch file named `spawn_valkyrie.launch.py`. You would typically run it like this:

```bash
ros2 launch valkyrie_description spawn_valkyrie.launch.py
```

This launch file will handle the complexity of converting the XACRO to URDF and calling the `spawn_entity.py` script with the correct parameters.

## Real-World Analogy

Importing a complex robot model is like downloading a highly detailed 3D character for a video game. The character comes with its skeleton (joints), its appearance (meshes), and maybe even some pre-set animations (controller configurations). You don't need to model it from scratch; you just need to know how to load it into the game engine (Gazebo).

## Hands-On Task

1.  Follow the steps above to clone and build the `valkyrie_description` package.
2.  Search through the package's `launch` directory to find a launch file that seems appropriate for spawning the robot in Gazebo.
3.  Launch Gazebo with an empty world.
4.  Run the launch file you found and watch the Valkyrie robot appear in the simulation.
5.  Use the Gazebo inspector (Ctrl+I) to click on different parts of Valkyrie and see its individual links.

## Common Mistakes & Debugging Tips

- **Missing Meshes:** If the robot appears in Gazebo but some parts are invisible, it often means the simulator couldn't find the mesh files. This can be a problem with incorrect paths in the URDF or environment variables not being set correctly. Ensure you have sourced your workspace (`source install/setup.bash`).
- **Build Failures:** `colcon build` can fail if you are missing dependencies. Carefully read the error messages and use `rosdep` or `apt` to install any missing packages.
- **`git clone` fails:** Make sure you have `git` installed (`sudo apt install git`) and that you have network connectivity.

## Mini Assessment

1.  Besides URDF files, what other important files are typically included in a robot description package?
2.  What is the purpose of the `rosdep` command?
3.  What is `colcon build` used for?
4.  True or False: You should always manually convert a complex robot's XACRO file to URDF before spawning.
5.  If a robot model loads but appears as a collection of disjointed, untextured shapes, what is the most likely problem?

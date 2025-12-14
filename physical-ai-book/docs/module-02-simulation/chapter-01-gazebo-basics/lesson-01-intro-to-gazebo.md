---
id: lesson-01-intro-to-gazebo
title: "Lesson 1: Introduction to Gazebo"
sidebar_position: 1
description: Your first steps into the world of robotics simulation with Gazebo.
---

## Lesson Objective

By the end of this lesson, you will be able to install Gazebo, understand its basic architecture, and launch a simple, pre-made simulation environment.

## Prerequisites

- A computer running a compatible Linux distribution (preferably Ubuntu 22.04).
- Basic familiarity with the Linux command line.

## Concept Explanation

Gazebo is a 3D robotics simulator that allows you to model complex robots and environments. It consists of two main components:
- **Gazebo Server (`gzserver`):** The backend that runs the physics engine, simulates sensors, and generates the state of the world.
- **Gazebo Client (`gzclient`):** The graphical frontend that visualizes the simulation and allows you to interact with it.

This separation allows you to run simulations on a powerful machine (the server) and visualize them on a less powerful one (the client), even over a network.

## Step-by-Step Technical Breakdown

### 1. Installation

First, add the Gazebo repository to your system:
```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```

Next, add the repository key:
```bash
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

Now, update your package lists and install Gazebo:
```bash
sudo apt-get update
sudo apt-get install gazebo11
```

### 2. Launching Gazebo

To launch both the server and the client, simply run:
```bash
gazebo
```

You should see a graphical window appear with an empty, grid-lined world.

### 3. Running a Demo World

Gazebo comes with several example worlds. Let's launch one:
```bash
gazebo worlds/pioneer2dx.world
```
This will load a world containing a simple wheeled robot.

## Real-World Analogy

Think of Gazebo as a hyper-realistic video game for robots. The `gzserver` is the game engine running on a powerful console, calculating all the physics and AI, while the `gzclient` is the screen you use to see and interact with the game world.

## Hands-On Task

Your task is to launch the `pioneer2dx.world` simulation. Once it's running, use your mouse to:
1. **Pan:** Hold the middle mouse button and move the mouse.
2. **Rotate:** Hold the left mouse button and move the mouse.
3. **Zoom:** Use the mouse scroll wheel.
Explore the simulated environment from different angles.

## Common Mistakes & Debugging Tips

- **"Command not found":** If you get a "gazebo: command not found" error, it means the installation was not successful or your shell can't find the executable. Re-run the installation steps.
- **Slow Performance:** Gazebo can be resource-intensive. If it runs slowly, ensure you have a decent GPU and that your system is not overloaded with other tasks. Closing the `gzclient` window will stop the visualization but leave the simulation running in the background, which can save resources.

## Mini Assessment

1. What are the two main components of Gazebo?
2. What command do you use to launch the Gazebo GUI?
3. How do you zoom in and out in the Gazebo client?
4. True or False: You must run the Gazebo client and server on the same machine.

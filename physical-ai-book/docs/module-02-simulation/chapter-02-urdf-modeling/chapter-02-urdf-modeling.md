---
id: chapter-02-urdf-modeling
title: "Chapter 2: URDF for Digital Twins"
sidebar_position: 2
description: Master the art of URDF to create detailed digital replicas of your robots.
---

## Summary

This chapter focuses on the Universal Robotic Description Format (URDF), the XML-based standard for representing a robot's physical structure. You'll learn how to build a robot model from scratch, defining its links, joints, and visual appearance to create an accurate digital twin.

## Why this chapter matters

A precise URDF model is the foundation of any realistic simulation. It dictates how the robot interacts with its environment, how its joints move, and what it "looks" like to the simulator's physics engine and rendering system. A well-defined URDF is crucial for accurate testing of everything from simple grasping to complex bipedal locomotion.

## Real Robotics Use-Cases

- **Industrial Manipulators:** Companies like ABB and KUKA use URDF to model their robotic arms, allowing for offline programming and collision detection.
- **Humanoid Robots:** Research labs developing humanoid robots like Atlas (Boston Dynamics) and Digit (Agility Robotics) rely on detailed URDFs to simulate and control their complex multi-jointed bodies.
- **Mobile Robots:** Delivery and exploration robots are modeled in URDF to simulate their interaction with varied terrains and obstacles.

## Skills Students Will Build

- Authoring URDF files from scratch.
- Defining links (the rigid parts of the robot) and joints (which connect links).
- Specifying visual meshes and collision geometries.
- Using XACRO to create modular and reusable URDFs.
- Spawning a URDF-based robot into a Gazebo simulation.

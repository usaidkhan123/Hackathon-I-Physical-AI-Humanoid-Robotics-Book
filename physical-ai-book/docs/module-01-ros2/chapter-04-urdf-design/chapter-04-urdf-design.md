---
id: chapter-04-urdf-design
title: Chapter 4 - URDF Design
sidebar_position: 4
description: Learn how to create a URDF file to describe the physical structure of a robot.
---

# Chapter 4: URDF Design

## Summary

This chapter introduces the Unified Robot Description Format (URDF), which is an XML format used to describe the physical structure of a robot. You will learn how to create a URDF file to define the links, joints, and sensors of a robot. You will also learn how to view and test your URDF model in RViz, the ROS 2 visualization tool.

## Why this chapter matters

A URDF file is a fundamental component of any ROS 2 system that involves a physical robot. It provides a standardized way to describe the robot's geometry, which is used by many ROS 2 tools for tasks such as visualization, collision checking, and kinematics.

## Real Robotics Use-Cases

- **Robot Simulation**: URDF files are used to create a model of a robot that can be used in a simulator like Gazebo.
- **Motion Planning**: Motion planning libraries, such as MoveIt, use URDF files to plan collision-free paths for a robot's arm.
- **Visualization**: RViz uses URDF files to display a 3D model of a robot, which is useful for debugging and monitoring.

## Skills Students Will Build

- **URDF Syntax**: Learn the syntax of the URDF format.
- **Robot Modeling**: Create a URDF file to describe a simple robot.
- **Visualization**: View and test a URDF model in RViz.

---
id: chapter-02-ros2-core
title: Chapter 2 - ROS 2 Core Concepts
sidebar_position: 2
description: A deep dive into the core concepts of ROS 2, including nodes, topics, services, and actions.
---

# Chapter 2: ROS 2 Core Concepts

## Summary

This chapter introduces the fundamental building blocks of ROS 2. You will learn how to create and manage ROS 2 nodes, which are the main executable programs in a ROS 2 system. You will also learn about the three main communication mechanisms in ROS 2: topics for asynchronous, one-to-many communication; services for synchronous, one-to-one communication; and actions for long-running, asynchronous tasks with feedback.

## Why this chapter matters

A solid understanding of ROS 2 core concepts is essential for building any robotics application with ROS 2. This chapter will provide you with the practical skills you need to start building your own ROS 2 systems and to understand the code of others.

## Real Robotics Use-Cases

- **Autonomous Navigation**: A robot's navigation stack might use topics to receive sensor data, services to request a path to a goal, and actions to execute the path.
- **Robotic Manipulation**: A robotic arm might use topics to publish its joint states, services to plan a grasp, and actions to execute a picking motion.
- **Multi-robot Systems**: ROS 2's communication mechanisms are designed to be distributed, making it possible to build systems of multiple robots that can communicate and coordinate with each other.

## Skills Students Will Build

- **ROS 2 Nodes**: Create and manage ROS 2 nodes using `rclpy`.
- **ROS 2 Topics**: Implement publishers and subscribers to communicate between nodes using topics.
- **ROS 2 Services**: Implement service clients and servers for request-response communication.
- **ROS 2 Actions**: Implement action clients and servers for long-running tasks.

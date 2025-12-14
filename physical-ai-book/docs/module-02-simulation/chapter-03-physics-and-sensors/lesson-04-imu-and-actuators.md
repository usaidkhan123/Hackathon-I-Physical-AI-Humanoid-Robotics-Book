---
id: lesson-04-imu-and-actuators
title: "Lesson 4: IMUs and Joint Control"
sidebar_position: 4
description: Simulate an IMU sensor and learn how to control robot joints directly.
---

## Lesson Objective

By the end of this lesson, you will be able to add a simulated Inertial Measurement Unit (IMU) to your robot and understand the basics of how to control a URDF joint using Gazebo plugins.

## Prerequisites

- Completion of the previous lessons in this chapter.
- A URDF with at least one non-fixed joint.

## Concept Explanation

### IMU Sensors

An **Inertial Measurement Unit (IMU)** is a crucial sensor for determining a robot's orientation and motion. It combines data from an accelerometer (measures linear acceleration) and a gyroscope (measures angular velocity). In simulation, an IMU plugin can provide ground-truth data about the link it's attached to. It publishes a `sensor_msgs/Imu` message.

### Joint Control in Gazebo

So far, we've used `joint_state_publisher` to manually move joints in RViz. To have a robot's joints be controlled by a program or another Gazebo plugin, we need a more robust interface. `gazebo_ros_control` (also known as `ros2_control`) is the standard way to do this, but for simple cases, we can use a basic plugin to directly set a joint's position or velocity.

## Step-by-Step Technical Breakdown

### 1. Adding an IMU Sensor

Let's add an IMU to our robot's `base_link`. Add the following inside a `<gazebo reference="base_link">` block in your URDF.

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <visualize>true</visualize>
  <imu>
    <angular_velocity>
      <x><noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev></noise></x>
      <y><noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev></noise></y>
      <z><noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev></noise></z>
    </angular_velocity>
    <linear_acceleration>
      <x><noise type="gaussian"><mean>0.0</mean><stddev>1.7e-2</stddev></noise></x>
      <y><noise type="gaussian"><mean>0.0</mean><stddev>1.7e-2</stddev></noise></y>
      <z><noise type="gaussian"><mean>0.0</mean><stddev>1.7e-2</stddev></noise></z>
    </linear_acceleration>
  </imu>
  <plugin name="imu_controller" filename="libgazebo_ros_imu_sensor.so">
    <ros>
      <namespace>/my_robot</namespace>
      <remapping>~/out:=imu</remapping>
    </ros>
    <initial_orientation_as_reference>false</initial_orientation_as_reference>
  </plugin>
</sensor>
```
- **`<sensor type="imu">`**: Defines the sensor as an IMU.
- **`<noise>`**: This is a key part of sensor simulation. We can add Gaussian noise to the measurements to make them more realistic.
- **`libgazebo_ros_imu_sensor.so`**: The plugin that simulates the IMU and publishes the `sensor_msgs/Imu` message to the `/my_robot/imu` topic.

### 2. Basic Joint Control Plugin

Let's make the `base_to_arm_joint` from our `simple_arm` model oscillate back and forth automatically. We'll use the built-in `JointController` plugin.

In your URDF, find the definition for `base_to_arm_joint` and add a `<gazebo>` block for it.

```xml
<gazebo reference="base_to_arm_joint">
  <plugin name="joint_controller" filename="libgazebo_ros_joint_controller.so">
    <ros>
      <namespace>/my_robot</namespace>
      <remapping>~/control_topic:=/my_robot/set_joint_position</remapping>
    </ros>
    <joint>base_to_arm_joint</joint>
    <initial_position>0.0</initial_position>
  </plugin>
</gazebo>
```
This plugin doesn't do anything by itself; it just creates a ROS 2 topic (`/my_robot/set_joint_position`) that we can publish to to set the joint's target position.

### 3. Controlling the Joint via Command Line

Now, launch the simulation. You can control the joint by publishing to the topic:
```bash
# Set the joint angle to 0.5 radians
ros2 topic pub /my_robot/set_joint_position std_msgs/msg/Float64 "data: 0.5"

# Set it back to -0.5 radians
ros2 topic pub /my_robot/set_joint_position std_msgs/msg/Float64 "data: -0.5"
```
You should see the arm move in Gazebo.

## Real-World Analogy

- **IMU:** This is the inner ear of your robot. It gives it a sense of balance, orientation, and acceleration, just like your own inner ear helps you stay upright.
- **Joint Controller Plugin:** This is like exposing the raw motor controls. You are providing a direct "voltage" or "position" command to a specific motor (joint) in the robot's body. A more advanced system (`ros2_control`) would be like the central nervous system, coordinating multiple motors together.

## Hands-On Task

1.  Add the IMU sensor to your robot's `base_link`.
2.  Launch the simulation. Manually move the robot around in the Gazebo GUI (using the translate tool).
3.  Use `ros2 topic echo /my_robot/imu` to see the IMU data change as you move the robot. Pay attention to the `linear_acceleration` when you start/stop moving it and the `angular_velocity` when you rotate it.
4.  Add the `joint_controller` plugin to your arm's revolute joint.
5.  Launch the simulation and use `ros2 topic pub` commands to make the arm swing back and forth between -1.0 and 1.0 radians.

## Common Mistakes & Debugging Tips

- **IMU Data Seems Wrong:** Remember that the IMU measures acceleration, not velocity. When the robot is stationary, the linear acceleration should be close to `(0, 0, 9.8)` due to gravity (if the IMU is oriented upright). When moving at a constant velocity, the acceleration will be near zero.
- **Joint Control Topic Doesn't Work:**
    - Double-check that the plugin is loaded and that you are publishing to the correct topic name.
    - Make sure you are publishing the correct message type (`std_msgs/msg/Float64`).
    - Verify that the joint is not `fixed`. The plugin can only control non-fixed joints.

## Mini Assessment

1.  What two physical phenomena does an IMU measure?
2.  What is the purpose of adding a `<noise>` block to a sensor definition?
3.  What ROS message type does the `libgazebo_ros_imu_sensor.so` plugin publish?
4.  The `libgazebo_ros_joint_controller.so` plugin allows you to control a joint by publishing to a ROS 2 ______.
5.  True or False: You can use the joint controller plugin on a `fixed` joint.

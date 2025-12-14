---
id: lesson-03-lidar-and-cameras
title: "Lesson 3: Lidar and Depth Cameras"
sidebar_position: 3
description: Simulate 3D perception sensors like LiDAR and Depth Cameras.
---

## Lesson Objective

By the end of this lesson, you will be able to add both a simulated 2D/3D LiDAR sensor and a depth camera to your URDF model and visualize their data in ROS 2.

## Prerequisites

- Completion of the previous lesson on adding a basic camera.
- A URDF file to modify.

## Concept Explanation

### LiDAR Sensors

**LiDAR (Light Detection and Ranging)** sensors measure distances by illuminating a target with a laser and analyzing the reflected light. They are a cornerstone of modern robotics for localization, mapping, and obstacle avoidance.
- **2D LiDAR:** Scans on a single plane, producing a `sensor_msgs/LaserScan` message.
- **3D LiDAR (or "Lidar"):** Scans on multiple planes, producing a `sensor_msgs/PointCloud2` message.

### Depth Cameras

A **depth camera** (or RGB-D camera) is like a regular camera but provides an additional channel for each pixel that contains the distance (depth) from the camera. This is extremely powerful for 3D perception. They also typically publish a `sensor_msgs/PointCloud2` message.

Both are added to a URDF using a `<sensor>` tag with the appropriate type and a corresponding plugin.

## Step-by-Step Technical Breakdown

### 1. Adding a 2D LiDAR Sensor

Let's add a 2D LiDAR to our `camera_link`. Add this inside the `<gazebo reference="camera_link">` block in your URDF.

```xml
<sensor type="ray" name="laser_sensor">
  <pose>0 0 0.1 0 0 0</pose>
  <visualize>true</visualize>
  <update_rate>40</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <resolution>1</resolution>
        <min_angle>-1.570796</min_angle>
        <max_angle>1.570796</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.10</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="gazebo_ros_laser_controller" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <namespace>/my_robot</namespace>
      <remapping>~/out:=scan</remapping>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
  </plugin>
</sensor>
```
- **`<sensor type="ray">`**: The type for laser-based sensors.
- **`<visualize>true</visualize>`**: Shows the laser beams in the Gazebo GUI.
- **`<scan>`**: Defines the properties of the scan (samples, resolution, angles).
- **`<range>`**: Defines the sensor's minimum and maximum range.
- **`libgazebo_ros_ray_sensor.so`**: The plugin that simulates the laser and publishes the data.
- **`<output_type>`**: Specifies we want a 2D `LaserScan` message.

### 2. Adding a Depth Camera

A depth camera is very similar to a regular camera, but uses the `libgazebo_ros_depth_camera.so` plugin.

```xml
<sensor type="depth" name="depth_camera">
    <update_rate>20.0</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R32F</format> <!-- 32-bit float for depth -->
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="depth_camera_controller" filename="libgazebo_ros_depth_camera.so">
      <ros>
        <namespace>/my_robot</namespace>
        <remapping>image_raw:=depth/image_raw</remapping>
        <remapping>camera_info:=depth/camera_info</remapping>
        <remapping>points:=depth/points</remapping>
      </ros>
      <camera_name>depth_camera</camera_name>
      <frame_name>camera_link</frame_name>
      <point_cloud_cutoff>0.4</point_cloud_cutoff>
    </plugin>
</sensor>
```
This plugin provides multiple outputs: a depth image (`/my_robot/depth/image_raw`) and a point cloud (`/my_robot/depth/points`).

### 3. Visualizing the Data in RViz

- **For the LaserScan:** Add a "LaserScan" display in RViz and set the topic to `/my_robot/scan`. You will see the laser points appear around your robot.
- **For the PointCloud2:** Add a "PointCloud2" display in RViz and set the topic to `/my_robot/depth/points`. You will see a 3D representation of what the depth camera sees.

## Real-World Analogy

- **2D LiDAR:** Imagine spinning around in a circle with a laser pointer and a stopwatch. You point the laser, time how long it takes for the dot to reflect back, and calculate the distance. You do this for a full circle to get a 2D map of your surroundings.
- **Depth Camera:** This is like having a grid of thousands of tiny laser pointers, all firing at once, to measure the distance to every point in an entire image simultaneously.

## Hands-On Task

1.  Add the 2D LiDAR sensor configuration to your robot's URDF.
2.  Launch the simulation and use RViz to visualize the `LaserScan` data. Place some simple shapes (boxes, spheres) in front of the robot in Gazebo and see them appear in the RViz visualization.
3.  Now, add the depth camera sensor to the URDF.
4.  Restart the simulation and use RViz to visualize the `/my_robot/depth/points` topic. You should see a 3D point cloud of the environment.

## Common Mistakes & Debugging Tips

- **No Point Cloud/LaserScan Topic:** As with the camera, this usually indicates a plugin loading error. Check the Gazebo terminal for any `*.so` related errors. Also, ensure `gazebo_ros_pkgs` is installed.
- **Points are at the wrong location:** This is a TF (transform) issue. Make sure your `robot_state_publisher` is running and that the `fixed_frame` in RViz is set correctly to your robot's root link (e.g., `base_link`). The sensor data is published relative to its own frame (`camera_link`), and RViz needs the transform tree to place it correctly in the world.

## Mini Assessment

1. What ROS message type does a 2D LiDAR typically publish?
2. What ROS message type does a 3D LiDAR or depth camera publish for 3D data?
3. In a URDF sensor definition, what does the `<visualize>true</visualize>` tag do?
4. What is the key difference between a regular camera and a depth camera's output?
5. What plugin is commonly used for simulating both 2D and 3D laser-based sensors?

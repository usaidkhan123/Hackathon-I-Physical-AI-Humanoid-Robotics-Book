---
id: lesson-02-gazebo-sensors
title: "Lesson 2: Adding a Camera Sensor"
sidebar_position: 2
description: Learn how to add a simulated camera to your robot and publish images to a ROS 2 topic.
---

## Lesson Objective

By the end of this lesson, you will be able to add a camera sensor to a URDF model, configure its properties, and view the simulated camera feed in ROS 2.

## Prerequisites

- Completion of the previous lesson.
- A URDF file for a robot.

## Concept Explanation

In Gazebo, sensors are implemented as plugins. To add a sensor, you attach it to a link in your URDF. The sensor plugin handles the generation of realistic sensor data and publishes it to a ROS 2 topic.

A sensor definition in URDF has two parts:
1.  The `<sensor>` tag: This defines the properties of the sensor itself, like its type, update rate, and sensor-specific parameters (e.g., image resolution for a camera).
2.  The `<plugin>` tag: This loads the Gazebo plugin that implements the sensor's behavior and the ROS 2 interface.

## Step-by-Step Technical Breakdown

### 1. Adding a Camera to a URDF

Let's add a camera to our `simple_arm` robot from the previous chapter. We'll attach it to the `arm_link`. Open the `.urdf.xacro` file and add the following inside the `<robot>` tags.

We need a new joint and link for the camera itself, to position it correctly relative to the arm.

```xml
<joint name="arm_to_camera_joint" type="fixed">
  <parent link="arm_link"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0" rpy="0 0 0"/>
</joint>

<link name="camera_link">
</link>
```

Now, attach the sensor to this new `camera_link`. This is a bit long, so we'll break it down.

```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="my_camera">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>800</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/my_camera</namespace>
        <remapping>image_raw:=image_raw</remapping>
        <remapping>camera_info:=camera_info</remapping>
      </ros>
      <camera_name>my_camera</camera_name>
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

- **`<gazebo reference="camera_link">`**: This tag tells Gazebo to attach the enclosed sensor description to the `camera_link`.
- **`<sensor type="camera" ...>`**: Defines a sensor of type `camera`.
- **`<update_rate>`**: How many times per second the sensor should generate data (30 Hz).
- **`<camera>`**: Contains camera-specific settings.
- **`<horizontal_fov>`**: The horizontal field of view in radians.
- **`<image>`**: The resolution and format of the output image.
- **`<clip>`**: The near and far clipping planes. Objects outside this range won't be rendered.
- **`<plugin name="camera_controller" ...>`**: This is the crucial part. It loads the `libgazebo_ros_camera.so` plugin, which does all the work.
- **`<ros>`**: This block configures the ROS 2 interface, setting the namespace and topic names.

### 2. Visualizing the Camera Feed

Now, launch your simulation with the updated robot model.

To see the camera feed, you can use `rqt_image_view` or add a Camera display in RViz.

```bash
# Using RQT Image View
ros2 run rqt_image_view rqt_image_view /my_camera/image_raw

# Or use RViz
rviz2
```
In RViz, add a "Camera" display and set the "Image Topic" to `/my_camera/image_raw`. You should see a real-time view from the perspective of your robot's camera.

## Real-World Analogy

Adding a sensor to your URDF is like mounting a webcam on a physical robot. The `<sensor>` tag is like choosing the webcam's hardware (e.g., a 1080p, 30fps camera). The `<plugin>` tag is like installing the driver for that webcam so that your computer's software (ROS 2) can access its video stream.

## Hands-On Task

1.  Add the camera sensor to your `simple_arm` URDF as described above.
2.  Launch the simulation and spawn the robot.
3.  Use `rqt_image_view` to view the camera feed.
4.  Move the arm's joint using the `joint_state_publisher_gui`. You should see the camera view move in `rqt_image_view`.
5.  Modify the `<image>` tag in your URDF to change the resolution to `1280x720` and restart the simulation to see the effect.

## Common Mistakes & Debugging Tips

- **No Image Topic:** If the `/my_camera/image_raw` topic doesn't appear, check the Gazebo terminal for errors. It often means the `libgazebo_ros_camera.so` plugin failed to load. Make sure `gazebo_ros_pkgs` is installed correctly.
- **Black Image:** A black or empty image could mean the camera is obstructed (e.g., inside another part of the robot) or its clipping planes (`<near>` and `<far>`) are set incorrectly. Use the Gazebo GUI to view the camera's frustum (the pyramid shape showing its field of view) to debug this.
- **Slow Performance:** High-resolution cameras at a high frame rate can be computationally expensive. If your simulation is slow, try reducing the camera's resolution or update rate.

## Mini Assessment

1.  What Gazebo tag is used to attach a sensor to a specific link?
2.  What does the `<update_rate>` in a sensor definition control?
3.  What plugin is used to simulate a camera and provide a ROS 2 interface?
4.  What ROS 2 tool can you use to quickly view a raw image topic?
5.  True or False: A single robot link can only have one sensor attached to it.

---
id: lesson-03-sensors-in-urdf
title: Lesson 3 - Sensors in URDF
sidebar_position: 3
description: Learn how to add sensor definitions to a URDF file.
---

# Lesson 3: Sensors in URDF

## Lesson Objective

By the end of this lesson, you will be able to add a `<sensor>` element to a URDF file to define a sensor, such as a camera or a laser scanner.

## Prerequisites

- Completion of Lesson 2: Humanoid Joints

## Concept Explanation

In addition to defining the physical structure of a robot, a URDF file can also be used to define the robot's sensors. A `<sensor>` element is used to define a sensor, and it can be placed within a `<link>` element to attach the sensor to that link. The `<sensor>` element can be used to specify the type of sensor (e.g., `camera`, `ray`), its position and orientation relative to the link, and its properties (e.g., the camera's resolution, the laser scanner's range).

## Step-by-Step Technical Breakdown

1.  **Add a `<sensor>` element**: We will add a `<sensor>` element to one of the links in our URDF file.
2.  **Set the sensor type**: We will set the `type` attribute of the sensor to `camera`.
3.  **Set the sensor origin**: We will set the `origin` of the sensor to specify its position and orientation relative to the link.
4.  **Add sensor-specific elements**: We will add elements to the `<sensor>` element to define the camera's properties, such as its resolution and field of view.

## Real-World Analogy

Think of the sensors on a self-driving car. The car has cameras, LiDAR, and radar sensors that are mounted in specific locations on the car's body. In a URDF file, we can define these sensors and their properties in the same way.

## Hands-On Task

Follow the step-by-step technical breakdown to add a camera sensor to your URDF file.

## Python + ROS2 Code Example

**sensor_robot.urdf:**
```xml
<?xml version="1.0"?>
<robot name="sensor_robot">

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.5" />
      </geometry>
    </visual>
  </link>

  <link name="camera_link" />

  <joint name="camera_joint" type="fixed">
    <parent link="base_link" />
    <child link="camera_link" />
    <origin xyz="0.25 0 0.25" rpy="0 0 0" />
  </joint>

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
        <alwaysOn>true</alwaysOn>
        <updateRate>0.0</updateRate>
        <cameraName>my_camera</cameraName>
        <imageTopicName>image_raw</imageTopicName>
        <cameraInfoTopicName>camera_info</cameraInfoTopicName>
        <frameName>camera_link</frameName>
        <hackBaseline>0.07</hackBaseline>
        <distortionK1>0.0</distortionK1>
        <distortionK2>0.0</distortionK2>
        <distortionK3>0.0</distortionK3>
        <distortionT1>0.0</distortionT1>
        <distortionT2>0.0</distortionT2>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```
*Note: The `<gazebo>` tag is specific to the Gazebo simulator. Other simulators may use different tags.*

## Common Mistakes & Debugging Tips

- **Simulator-specific tags**: The way you define a sensor in a URDF file can be specific to the simulator you are using. Make sure you are using the correct tags for your simulator.
- **Incorrect sensor properties**: The properties of a sensor, such as its resolution and field of view, will affect how it behaves in the simulator. Make sure you set these properties to reasonable values.

## Mini Assessment

1.  What is a `<sensor>` element used for?
2.  Where is a `<sensor>` element placed in a URDF file?
3.  What are some of the properties that can be defined for a camera sensor?
4.  Why is it important to use the correct simulator-specific tags when defining a sensor?

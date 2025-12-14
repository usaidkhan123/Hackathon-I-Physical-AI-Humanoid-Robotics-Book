---
id: lesson-02-setting-up-unity-ros
title: "Lesson 2: Setting up Unity for ROS 2"
sidebar_position: 2
description: A guide to installing and configuring the necessary packages to connect Unity with ROS 2.
---

## Lesson Objective

By the end of this lesson, you will be able to install the required Unity packages and ROS 2 packages to establish communication between a Unity scene and your ROS 2 environment.

## Prerequisites

- Unity Hub and a recent version of the Unity Editor (e.g., 2022.3 LTS) installed.
- A working ROS 2 Humble installation.
- Basic familiarity with the Unity Editor interface.

## Concept Explanation

Communication between Unity and ROS 2 is typically achieved through a "bridge" that relays messages between the two systems. The official and most common way to do this is using the **Unity Robotics Hub** packages.

The architecture looks like this:

1.  **Unity Scene:** Your robot and environment exist within a Unity project.
2.  **ROS TCP Connector (Unity Package):** A package within Unity that handles the serialization and deserialization of ROS messages. It acts as a TCP client.
3.  **ROS TCP Endpoint (ROS 2 Package):** A ROS 2 node that runs on your machine and acts as a TCP server. It subscribes to ROS topics and forwards the messages to the Unity client, and it receives messages from Unity and publishes them onto ROS topics.

This setup allows your ROS 2 nodes to be completely unaware that they are communicating with a simulation in Unity, rather than a physical robot.

## Step-by-Step Technical Breakdown

### 1. Install ROS 2 Packages

First, you need to install the ROS 2 side of the bridge, the `ros-tcp-endpoint`.
```bash
sudo apt-get update
sudo apt-get install ros-humble-ros-tcp-endpoint
```

### 2. Create a New Unity Project

1.  Open Unity Hub and create a new 3D project.
2.  Give it a name, for example, "MyRoboticsProject".

### 3. Install the Unity Robotics Hub Package

1.  In the Unity Editor, go to `Window -> Package Manager`.
2.  Click the `+` icon in the top left and select "Add package from git URL...".
3.  Enter the following URL: `https://github.com/Unity-Technologies/Unity-Robotics-Hub.git?path=/com.unity.robotics.ros-tcp-connector`
4.  Unity will download and install the package. This will give you the core ROS-TCP connection functionality.

### 4. Configure the ROS Connection

1.  In your Unity scene, create an empty GameObject and name it "ROSConnection".
2.  With the `ROSConnection` object selected, go to the Inspector and click "Add Component".
3.  Search for "ROS TCP Connector" and add it.
4.  In the `ROS TCP Connector` component, you can set the `ROS IP Address`. If you are running the ROS 2 endpoint on the same machine, you can leave it as the default `127.0.0.1`.
5.  You can also specify the port, which should match the port used by the ROS 2 endpoint (default is 10000).

### 5. Running the ROS 2 Endpoint

Now, you need to start the server that Unity will connect to. Open a terminal and run:
```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1
```
- `default_server_endpoint`: This is the main executable that listens for connections.
- `--ros-args -p ROS_IP:=127.0.0.1`: This sets the ROS IP parameter for the endpoint.

### 6. Establishing the Connection

Go back to Unity and enter Play mode by clicking the "Play" button at the top of the editor. If everything is configured correctly, you should see messages in both the Unity Console and the terminal running the ROS 2 endpoint indicating that a connection has been established.

## Hands-On Task

1.  Follow the steps above to create a new Unity project.
2.  Install the `ros-tcp-endpoint` package on your system.
3.  Install the Unity Robotics Hub package into your Unity project.
4.  Configure the `ROSConnection` GameObject in your scene.
5.  Start the `default_server_endpoint` in your terminal.
6.  Enter Play mode in Unity and verify that a connection is established. You should see a "Successfully connected to ROS" message in the Unity console.

## Common Mistakes & Debugging Tips

- **Connection Refused:** If Unity fails to connect, the most likely causes are:
    - The ROS 2 endpoint is not running. Make sure the `ros2 run` command is active in a terminal.
    - There is a firewall blocking the connection. Temporarily disable your firewall to check if this is the issue.
    - The IP address or port in the Unity `ROS TCP Connector` does not match the one the ROS 2 endpoint is listening on. Double-check that they are identical.
- **Package Not Found:** If Unity can't find the package from the git URL, ensure you have `git` installed on your system and that you have a working internet connection.

## Mini Assessment

1.  What is the name of the Unity package that handles ROS communication?
2.  What is the name of the ROS 2 package that acts as the server for Unity to connect to?
3.  What protocol is used for communication between Unity and ROS 2 in this setup?
4.  True or False: You must run the ROS 2 endpoint on the same machine as Unity.
5.  In Unity, where do you configure the IP address of the ROS 2 endpoint?

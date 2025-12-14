---
id: lesson-03-importing-urdf-to-unity
title: "Lesson 3: Importing URDF to Unity"
sidebar_position: 3
description: Learn how to import a URDF model into your Unity scene.
---

## Lesson Objective

By the end of this lesson, you will be able to import a URDF file into Unity, have Unity automatically create the corresponding robot prefab, and understand the basic components that make up the imported robot.

## Prerequisites

- Completion of the previous lesson.
- A URDF file to import (e.g., the `simple_arm.urdf` created in a previous chapter).

## Concept Explanation

The **Unity Robotics Hub** provides a tool to parse URDF files and automatically generate a corresponding **Prefab** in Unity. A Prefab is a reusable GameObject that stores all its components and properties. This allows you to configure a robot once and then easily instantiate it in different scenes.

The importer performs several key steps:
1.  **Parses the URDF:** It reads the XML structure of the URDF file.
2.  **Creates GameObjects:** It creates a hierarchy of GameObjects to represent the links of the robot.
3.  **Imports Meshes:** It finds the mesh files referenced in the `<visual>` tags and imports them into the Unity project.
4.  **Creates Joints:** It adds `ArticulationBody` components to the GameObjects to represent the joints defined in the URDF. `ArticulationBody` is a specialized Unity component designed for creating robotic arms and kinematic chains.
5.  **Assigns Materials:** It applies materials to the visual meshes.

## Step-by-Step Technical Breakdown

### 1. Install the URDF Importer Package

The URDF importer is part of the Unity Robotics Hub, but it's a separate package.
1.  In the Unity Editor, go to `Window -> Package Manager`.
2.  Click the `+` icon and select "Add package from git URL...".
3.  Enter the URL: `https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer`
4.  Unity will install the package. You should now have a new menu item: `Robotics -> URDF Importer`.

### 2. Import a URDF File

1.  From the Unity menu, select `Robotics -> URDF Importer`.
2.  A window will pop up. Drag and drop your URDF file (e.g., `simple_arm.urdf.xacro`, the importer can handle XACRO) into the designated field.
3.  You can leave the import settings as default for now.
4.  Click the "Import" button.

Unity will process the file and create a new folder in your `Assets` directory with the name of your robot. Inside this folder, you will find:
- A **Prefab** of your robot.
- Meshes, if there were any.
- Materials.

### 3. Add the Robot to Your Scene

1.  Drag the newly created robot Prefab from the Project window into your Scene view or the Hierarchy window.
2.  The robot will appear in your scene.

### 4. Inspecting the Imported Robot

Click on the robot in the Hierarchy to inspect it. You will see a parent GameObject with several children, each representing a link.
-   **`ArticulationBody`:** Look at the GameObjects corresponding to your moving links (e.g., `arm_link`). You will see they have an `ArticulationBody` component. This component defines the joint type (Revolute, Prismatic, etc.) and its motion ranges (limits).
-   **`URDFLink` script:** Each link will have a `URDFLink` script attached, which just holds information about the original URDF link.

## Hands-On Task

1.  Make sure you have the `simple_arm.urdf.xacro` file available from Chapter 2.
2.  Install the URDF Importer package in your Unity project.
3.  Use the importer to import the `simple_arm.urdf.xacro` file.
4.  Drag the generated `simple_arm` prefab into your scene.
5.  Select the `arm_link` GameObject and inspect its `ArticulationBody` component. Note how the joint type is set to "Revolute".
6.  Enter Play mode. The robot arm should fall down due to gravity.

## Common Mistakes & Debugging Tips

-   **Import Fails:** If the URDF import fails, it's almost always due to an error in the URDF/XACRO file itself. The Unity console will often provide a detailed error message from the `xacro` command. Try running `xacro your_file.urdf.xacro` on the command line to debug it.
-   **Missing Meshes:** If your URDF references mesh files (e.g., in `.dae` or `.stl` format), make sure they are located in a path relative to the URDF file so the importer can find them. It's best to keep them in the same directory.
-   **Robot Explodes on Play:** If the robot behaves erratically or "explodes" when you press Play, it could be due to:
    -   Poorly defined inertial properties (mass and inertia tensor) in the URDF.
    -   Inter-penetrating collision meshes. The importer has a tool to generate convex collision meshes which can help with this.

## Mini Assessment

1.  What is a Unity "Prefab"?
2.  What Unity component is used to represent robot joints imported from a URDF?
3.  True or False: The Unity URDF importer can directly parse `.urdf.xacro` files.
4.  Where in the Unity menu do you find the URDF Importer?
5.  If a URDF has a `<mesh>` tag, what does the importer do?

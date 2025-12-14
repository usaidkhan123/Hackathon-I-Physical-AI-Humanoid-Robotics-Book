---
id: lesson-01-simulating-physics
title: "Lesson 1: Simulating Physics"
sidebar_position: 1
description: Learn how to configure the physics properties of your Gazebo world and models.
---

## Lesson Objective

By the end of this lesson, you will be able to configure global physics properties in a Gazebo world file and define material properties like friction and restitution for individual models.

## Prerequisites

- Completion of Chapter 1: Gazebo Basics.
- A URDF model to experiment with.

## Concept Explanation

Gazebo's realism comes from its underlying physics engine. By default, it uses the **Open Dynamics Engine (ODE)**, but it can also be configured to use others like Bullet or DART.

You can control physics at two levels:
1.  **Global Physics:** In the `.world` file, you can set properties like gravity, the simulation step size, and the engine's solver parameters.
2.  **Local Physics (Contact Properties):** For each link's `<collision>` tag in a URDF, you can define how it interacts with other objects. This is done through the `<surface>` tag, which controls properties like friction and bounciness.

## Step-by-Step Technical Breakdown

### 1. Configuring Global Physics

Open a `.world` file, like the `my_world.world` from Chapter 1. The `<physics>` block controls the global settings.

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <gravity>0 0 -9.81</gravity>
</physics>
```
- **`<max_step_size>`**: The duration of each simulation step in seconds. Smaller values increase accuracy but require more computation.
- **`<real_time_factor>`**: A target for how fast the simulation should run relative to real time. `1` means real time, `0` means as fast as possible.
- **`<real_time_update_rate>`**: How often Gazebo tries to sync the simulation time with real time (in Hz).

### 2. Defining Contact Properties

Now, let's make our `blue_box` from Chapter 1 bouncy. In its URDF file, modify the `<collision>` tag for its link.

```xml
<collision name="collision">
  <geometry>
    <box size="1 1 1"/>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>0.1</mu> <!-- Coefficient of friction -->
        <mu2>0.1</mu2>
      </ode>
    </friction>
    <contact>
      <ode>
        <kp>1000000.0</kp> <!-- Contact stiffness -->
        <kd>1.0</kd> <!-- Contact damping -->
        <max_vel>0.01</max_vel> <!-- Max velocity at contact -->
        <min_depth>0.001</min_depth> <!-- Min depth for contact -->
      </ode>
    </contact>
    <bounce>
      <restitution_coefficient>0.5</restitution_coefficient>
      <threshold>1.0</threshold>
    </bounce>
  </surface>
</collision>
```
- **`<friction>`**: Defines the static and dynamic friction coefficients (`mu` and `mu2`).
- **`<contact>`**: Defines how the physics engine should handle contact forces.
- **`<bounce>`**: Defines the "bounciness" of the surface. A `restitution_coefficient` of `1.0` would be perfectly elastic.

### 3. Spawning the Model

Launch your world, and then spawn your model from a height:
```bash
ros2 run gazebo_ros spawn_entity.py -entity my_bouncy_box -file my_box.urdf -z 5
```
Watch the box fall. It should now bounce a few times before coming to rest.

## Real-World Analogy

Configuring physics is like setting the laws of your own private universe. You can create a world with the gravity of Mars, make objects out of super-ice (`mu=0`), or create a trampoline out of "bouncy" ground.

## Hands-On Task

1.  Create a new world named `physics_playground.world`.
2.  In this world, set the gravity to be half of Earth's gravity (`0 0 -4.9`).
3.  Create two boxes in this world:
    *   A "slippery" box with a very low friction coefficient (`mu=0.05`).
    *   A "sticky" box with a very high friction coefficient (`mu=100.0`).
4.  Create a sloped ground plane using the `<plane>` geometry with a tilt.
5.  Place both boxes on the slope and observe the difference in their behavior. The slippery box should slide down easily, while the sticky box should not.

## Common Mistakes & Debugging Tips

- **Unrealistic Behavior:** If your robot model is "jittery" or flies apart, it's often a sign of unstable physics. This can be caused by bad inertial values (mass, inertia), or physics properties that are too extreme. Try reducing the `max_step_size` or adjusting the contact `kp` and `kd` values.
- **Friction Not Working:** Friction is calculated between two surfaces. If one of the surfaces has a friction of zero, the contact will be frictionless. Make sure both the robot's link and the ground plane have friction defined.

## Mini Assessment

1.  What physics engine does Gazebo use by default?
2.  What does the `<real_time_factor>` control?
3.  What tag is used to define the bounciness of an object?
4.  What is the `mu` parameter in the `<friction>` tag?
5.  True or False: You can set different gravity for different models in the same world.

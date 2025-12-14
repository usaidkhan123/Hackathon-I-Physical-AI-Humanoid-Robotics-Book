---
id: lesson-02-global-local-path-planning
title: "Lesson 2: Global and Local Path Planning"
sidebar_position: 2
description: Understand and configure Nav2's global and local planners to generate efficient, collision-free paths for humanoid robots.
---

# Lesson 2: Global and Local Path Planning

## Lesson Objectives
- Differentiate between global and local path planning in Nav2.
- Learn to select and configure various global planners (e.g., A*, Dijkstra) and local planners (e.g., DWB, TEB).
- Optimize planner parameters for specific robot kinematics and environmental conditions.
- Visualize and debug global and local plans in RViz 2.

## Prerequisites
- Completion of Lesson 1: Nav2 Fundamentals and Setup.
- Basic understanding of search algorithms and robot motion models.
- Familiarity with Nav2 configuration files (`yaml`).

## Concept Explanation
Nav2 utilizes a two-tiered planning approach: global and local. The **global planner** computes a high-level, long-term path from the robot's current location to the goal, typically avoiding large obstacles on a static map. The **local planner** (also known as the local controller or trajectory planner) then takes small segments of this global path and continuously generates velocity commands to follow it, while performing dynamic obstacle avoidance and ensuring the robot's motion constraints are met. Understanding both is crucial for efficient and safe autonomous navigation.

## Step-by-Step Technical Breakdown

1.  **Global Planners**: Explore popular global planners like NavFn (Dijkstra/A* based), Theta*, and SmacPlanner (hybrid A*). Configure their parameters in Nav2's YAML files (e.g., `planner_server.yaml`).
2.  **Local Planners**: Investigate local planners such as DWB (Dynamic Window Bouncing), TEB (Timed-Elastic Band), and Regulated Pure Pursuit. Configure their parameters (e.g., velocity limits, acceleration limits, path following weights) in `controller_server.yaml`.
3.  **Costmaps**: Understand how global and local costmaps are generated and used by the planners. Configure layers (static map, obstacle, inflation) and their parameters in `costmap_filters.yaml` and `global_costmap.yaml`/`local_costmap.yaml`.
4.  **Tuning for Humanoids**: Adjust planner parameters specifically for humanoid robots, considering their unique kinematics (e.g., bipedal walking, balance control) and motion capabilities, which differ from wheeled robots.

## Real-World Analogy
Think of planning a road trip. The **global planner** is like using Google Maps to find the best route from one city to another, avoiding major highways that are closed. The **local planner** is like your actual driving: constantly adjusting steering and speed to stay in your lane, avoid other cars, and react to traffic lights, all while aiming for the next turn on your Google Maps route.

<h2>Hands-On Tasks</h2>
1.  Launch your Nav2 setup from Lesson 1.
2.  Experiment with different global planners by changing the `default_planner` parameter in your `nav2_params.yaml` (or equivalent) and observing their behavior with various goals.
3.  Similarly, experiment with different local planners (`default_controller`) and tune parameters like `max_vel_x`, `min_vel_x`, and `max_vel_theta` to suit your robot's motion.
4.  Introduce dynamic obstacles in your simulated environment and observe how the local planner reacts and avoids them.
5.  (Advanced) Try to implement a simple "footstep planner" (conceptual) for a bipedal humanoid that integrates with the local planner to generate valid foot placements.

<h2>Python + ROS2 Examples (Conceptual Nav2 Configuration Snippet)</h2>

```yaml
# Snippet from a conceptual nav2_params.yaml for planner configuration

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001

    progress_checker_plugin: "progress_checker::ProgressChecker"
    goal_checker_plugin: "goal_checker::RotationOnlyGoalChecker"
    controller_plugins: ["FollowPath"] # DWB is common, or custom

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner" # Example using DWB
      # DWB specific parameters (simplified)
      max_vel_x: 0.5
      min_vel_x: -0.1
      max_vel_theta: 1.0
      min_vel_theta: -1.0
      acc_lim_x: 2.5
      acc_lim_theta: 5.0
      prune_plan: True
      # More DWB parameters for trajectory generation, path distance, goal distance, etc.

planner_server:
  ros__parameters:
    use_sim_time: True
    planner_plugins: ["GridBased"] # NavFn, ThetaStar, or SmacPlanner

    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner" # Example using NavFn
      # NavFn specific parameters (simplified)
      allow_unknown: True
      default_tolerance: 0.2
      # More NavFn parameters for cost computations, etc.

recovery_server:
  ros__parameters:
    use_sim_time: True
    recovery_plugins: ["spin", "backup", "clear_costmap"]
    # ... configurations for each recovery plugin
```

## Debugging Tips
-   **Parameter Sensitivity**: Nav2 parameters are highly interdependent. Change one parameter at a time and observe its effect.
-   **RViz 2 Costmaps**: Visualize global and local costmaps. If the robot isn't moving where expected, the costmap might be misconfigured (e.g., too much inflation).
-   **Planner Output**: In RViz 2, display the global plan and local plan. Does the global plan make sense? Is the local plan following it correctly?
-   **Humanoid Motion Constraints**: Ensure the `max_vel_x`, `max_vel_theta`, and acceleration limits are appropriate for your humanoid robot's physical capabilities.

## Mini Quiz (4-6 questions)
1.  What is the primary function of the global planner in Nav2?
2.  How does the local planner differ from the global planner in its operation?
3.  Name two popular local planners available in Nav2.
4.  What are costmaps, and how do they influence path planning?
5.  Why is it important to tune Nav2 parameters specifically for humanoid robots?
```
---
id: lesson-01-nav2-fundamentals-setup
title: "Lesson 1: Nav2 Fundamentals and Setup"
sidebar_position: 1
description: "Get started with the ROS 2 Navigation Stack (Nav2), understand its architecture, and set up a basic navigation system for your robot."
---

# Lesson 1: Nav2 Fundamentals and Setup

## Lesson Objectives
- Understand the high-level architecture of the Nav2 stack.
- Learn to configure essential Nav2 components, including global and local planners.
- Set up a basic navigation system for a simulated robot in a known environment.
- Send navigation goals and monitor the robot's progress using Nav2.

## Prerequisites
- Basic understanding of ROS 2 concepts (nodes, topics, services, actions).
- Familiarity with a robotic simulator (e.g., Gazebo, Isaac Sim).
- A robot model with odometry and a map available.

## Concept Explanation
Nav2 is the standard ROS 2 navigation stack, providing a comprehensive suite of tools for autonomous mobile robot navigation. It takes a map of the environment, sensor data (like LiDAR or cameras), and a goal pose, then plans a path and executes the robot's movement while avoiding obstacles. Nav2 is highly modular, allowing for flexible configuration of various planners, controllers, and recovery behaviors, making it adaptable to diverse robot platforms, including humanoid robots.

## Step-by-Step Technical Breakdown

1.  **Nav2 Architecture**: Review the key components of Nav2: `map_server`, `amcl` (Adaptive Monte Carlo Localization), `global_planner`, `local_planner` (e.g., DWB, TEB), `recovery_behaviors`, and `bt_navigator` (Behavior Tree Navigator).
2.  **Launch Nav2**: Use a Nav2 launch file to start all necessary nodes. This typically involves:
    *   Loading a pre-built map.
    *   Starting `amcl` for localization.
    *   Configuring the global and local planners.
3.  **Map Preparation**: Provide Nav2 with an occupancy grid map of the environment (e.g., `map.yaml` and `map.pgm` files). This map can be generated from SLAM (like Isaac ROS VSLAM) or pre-drawn.
4.  **Localization**: Initialize the robot's pose on the map (e.g., using RViz 2's "2D Pose Estimate" tool).
5.  **Send Goals**: Use RViz 2's "2D Goal Pose" tool or a ROS 2 command-line tool to send navigation goals to the robot.

## Real-World Analogy
Imagine giving GPS coordinates (a goal) to a self-driving car (robot). Nav2 is like the car's entire navigation system: it knows where it is (localization), has a map of the roads (global planner), decides how to steer and brake to stay on track and avoid immediate obstacles (local planner), and knows what to do if it gets stuck (recovery behaviors).

<h2>Hands-On Tasks</h2>
1.  Launch a simulated robot in an environment with a known map (e.g., TurtleBot3 in a Gazebo world, or a custom environment in Isaac Sim).
2.  Obtain or create an occupancy grid map of this environment.
3.  Write a ROS 2 launch file to bring up the core Nav2 stack components, loading your map.
4.  Launch RViz 2, add the Map, RobotModel, and Nav2 plugins.
5.  Use RViz 2 to give the robot an initial pose estimate and then send it several navigation goals. Observe its path planning and movement.

<h2>Python + ROS2 Examples (Conceptual Nav2 Launch File)</h2>

```bash
# This is a conceptual example of a Nav2 launch file.
# The exact parameters will vary based on your robot and environment.

# In your ROS 2 package (e.g., my_robot_nav/launch/nav2_bringup_launch.py)
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths to your map and Nav2 configuration files
    map_file_path = LaunchConfiguration('map', 
                                        default=PathJoinSubstitution([
                                            get_package_share_directory('my_robot_nav'), # Your package
                                            'maps', 
                                            'my_environment_map.yaml' # Your map file
                                        ]))
    nav2_config_path = LaunchConfiguration('params_file',
                                           default=PathJoinSubstitution([
                                               get_package_share_directory('my_robot_nav'), # Your package
                                               'config',
                                               'nav2_params.yaml' # Your Nav2 config file
                                           ]))
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_file_path,
            description='Full path to map yaml file to load'),
        DeclareLaunchArgument(
            'params_file',
            default_value=nav2_config_path,
            description='Full path to the ROS2 parameters file to use for all launched nodes'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        # Include the main Nav2 launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    get_package_share_directory('nav2_bringup'),
                    'launch',
                    'bringup_launch.py'
                ])
            ),
            launch_arguments={
                'map': map_file_path,
                'params_file': nav2_config_path,
                'use_sim_time': use_sim_time,
            }.items()
        ),
        
        # Optional: RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                get_package_share_directory('nav2_bringup'),
                'rviz',
                'nav2_default_view.rviz' # Or your custom RViz config
            ])],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])
```

<h2>Debugging Tips</h2>
-   **TF Tree**: A common source of Nav2 issues is an incorrect TF tree. Use `ros2 run tf2_tools view_frames` to visualize your TF tree and ensure all frames are correctly connected.
-   **Map Quality**: Ensure your occupancy grid map is accurate and free of artifacts.
-   **Odometry**: Verify that your robot's odometry source is providing accurate and consistent pose estimates.
-   **Parameter Tuning**: Nav2 has many parameters. Start with default parameters and incrementally tune them. `rqt_reconfigure` can be very helpful here.
-   **Visualization**: RViz 2 is essential. Visualize the map, robot pose, global plan, local plan, and costmaps to understand what Nav2 is "seeing" and "planning."

<h2>Mini Quiz (4-6 questions)</h2>
1.  What is the primary function of the Nav2 stack in ROS 2?
2.  Name three core components of the Nav2 architecture.
3.  How does a robot typically provide its initial position to Nav2?
4.  What type of map does Nav2 primarily use for navigation?
5.  What is the role of the global planner in Nav2?
```
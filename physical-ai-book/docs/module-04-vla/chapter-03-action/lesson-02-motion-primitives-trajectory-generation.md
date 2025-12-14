---
id: lesson-02-motion-primitives-trajectory-generation
title: "Lesson 2: Motion Primitives and Trajectory Generation"
sidebar_position: 2
description: Translate high-level LLM actions into low-level robot joint commands and smooth motion trajectories.
---

# Lesson 2: Motion Primitives and Trajectory Generation

## Lesson Objectives
- Understand the concept of motion primitives and their role in robot control.
- Learn how to use inverse kinematics to translate end-effector poses into joint commands.
- Implement basic trajectory generation for smooth and safe robot movements.

## Prerequisites
- Completion of Lesson 1: ROS 2 Action Servers for LLM Commands.
- Basic understanding of robot kinematics (forward and inverse).
- Familiarity with common ROS 2 motion planning frameworks (e.g., MoveIt 2).

## Concept Explanation
The LLM generates high-level plans using abstract actions like "grasp_object" or "move_to_location." These commands need to be translated into the specific joint angles and movements that a robot's motors can execute. Motion primitives are pre-defined, parameterized atomic movements (e.g., "reach," "retract," "pick," "place"). Trajectory generation then smooths these movements into a continuous, collision-free path, often involving inverse kinematics to calculate the required joint configurations for desired end-effector positions.

<h2>Step-by-Step Technical Breakdown</h2>
1.  **Inverse Kinematics (IK)**: Given a desired end-effector position and orientation (pose), calculate the corresponding joint angles for the robot's arm. Libraries like `KDL` or `TRAC-IK` integrated with MoveIt 2 are commonly used.
2.  **Trajectory Planning**: Using MoveIt 2 (or a similar framework), plan a collision-free path from the robot's current configuration to the target joint configuration. This involves avoiding obstacles in the environment.
3.  **Motion Execution**: Send the generated trajectory (sequence of joint states over time) to the robot's joint controllers.
4.  **Integration with Action Server**: The ROS 2 Action Server (from Lesson 1) will call these motion primitive functions as part of executing a high-level goal.

<h2>Real-World Analogy</h2>
If you want to pick up a pen, your brain doesn't tell each muscle to contract a specific amount. Instead, it issues a high-level command ("reach for the pen"). Your motor system then figures out the exact muscle movements (inverse kinematics) and smooths them out (trajectory generation) to make the action fluid and precise.

<h2>Hands-On Tasks</h2>
1.  Set up a simulated robot in Gazebo or RViz with MoveIt 2 configured for its arm.
2.  Use MoveIt 2's Python interface (or a direct IK solver) to calculate joint angles for a few target end-effector poses (e.g., above a table, grasping position).
3.  Implement a simple ROS 2 Service that takes an end-effector pose as input and attempts to plan and execute a movement to that pose using MoveIt 2.
4.  Integrate this service into your `NavigateToActionServer` (or a new `GraspObjectActionServer`) as part of its execution logic.

<h2>Python + ROS2 Examples</h2>

```python
# motion_primitive_client.py (Example ROS 2 Client using MoveIt 2)
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetMotionPlan, ApplyPlanningScene
from std_msgs.msg import Header
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time

class MotionPrimitiveClient(Node):
    def __init__(self):
        super().__init__('motion_primitive_client')
        self.callback_group = ReentrantCallbackGroup() # Allows parallel callbacks
        self.get_logger().info('Motion Primitive Client started.')

        self.motion_plan_client = self.create_client(
            GetMotionPlan,
            '/plan_kinematic_path',
            callback_group=self.callback_group
        )
        self.apply_planning_scene_client = self.create_client(
            ApplyPlanningScene,
            '/apply_planning_scene',
            callback_group=self.callback_group
        )

        while not self.motion_plan_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Motion planning service not available, waiting again...')
        while not self.apply_planning_scene_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Apply planning scene service not available, waiting again...')
        
        self.get_logger().info('MoveIt services connected.')

    def plan_and_execute_pose(self, target_pose: PoseStamped):
        self.get_logger().info(f"Attempting to move to target pose: {target_pose.pose.position.x}, {target_pose.pose.position.y}, {target_pose.pose.position.z}")
        
        request = GetMotionPlan.Request()
        request.motion_plan_request.group_name = 'arm' # Replace with your robot's arm group name
        request.motion_plan_request.num_planning_attempts = 10
        request.motion_plan_request.allowed_planning_time = 5.0

        request.motion_plan_request.start_state.is_diff = True # Use current robot state
        
        # Set target pose as a pose goal
        pose_goal = request.motion_plan_request.goal_constraints.add()
        pose_goal.link_name = 'gripper_link' # Replace with your robot's end-effector link name
        pose_goal.pose.header.frame_id = 'base_link' # Replace with your robot's base frame
        pose_goal.pose = target_pose.pose
        pose_goal.tolerance_pos.x = 0.005 # 5mm tolerance
        pose_goal.tolerance_pos.y = 0.005
        pose_goal.tolerance_pos.z = 0.005
        pose_goal.tolerance_ori.x = 0.01 # Orientation tolerance
        pose_goal.tolerance_ori.y = 0.01
        pose_goal.tolerance_ori.z = 0.01

        self.get_logger().info('Requesting motion plan...')
        future = self.motion_plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response.motion_plan_response.trajectory.joint_trajectory.points:
            self.get_logger().info('Motion plan found! Attempting to execute...')
            
            # Here you would typically send the trajectory to the robot's controllers
            # For simulation, MoveIt 2 can execute the plan directly
            # This part requires a more involved setup with MoveIt's planning_scene_monitor
            # and potentially a dedicated execution client.
            # For this example, we'll just log success.
            self.get_logger().info('Simulating execution of trajectory.')
            return True
        else:
            self.get_logger().warn('Failed to find a motion plan.')
            return False

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    motion_client = MotionPrimitiveClient()
    executor.add_node(motion_client)

    target_pose = PoseStamped()
    target_pose.header = Header()
    target_pose.header.stamp = motion_client.get_clock().now().to_msg()
    target_pose.header.frame_id = 'base_link' 
    target_pose.pose.position.x = 0.3
    target_pose.pose.position.y = 0.0
    target_pose.pose.position.z = 0.5
    target_pose.pose.orientation.w = 1.0 # No rotation

    motion_client.plan_and_execute_pose(target_pose)

    motion_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

<h2>Debugging Tips</h2>
-   **Robot Model Setup**: Ensure your URDF/XACRO model is correctly loaded and configured for MoveIt 2, including joint limits and collision geometries.
-   **Inverse Kinematics Failures**: If IK fails, check the target pose for reachability. Some poses might be kinematically impossible for the robot.
-   **Collision Objects**: When planning in a cluttered environment, ensure all collision objects are correctly added to the planning scene.
-   **MoveIt 2 RViz Plugin**: Use the MoveIt 2 RViz plugin to visualize planning requests, planned paths, and detected collisions.

<h2>Mini Quiz (4-6 questions)</h2>
1.  What is a motion primitive in the context of robot control?
2.  What does inverse kinematics (IK) help a robot achieve?
3.  Why is trajectory generation important for smooth robot movements?
4.  Which ROS 2 framework is commonly used for motion planning and execution?
5.  What can cause an inverse kinematics solver to fail?

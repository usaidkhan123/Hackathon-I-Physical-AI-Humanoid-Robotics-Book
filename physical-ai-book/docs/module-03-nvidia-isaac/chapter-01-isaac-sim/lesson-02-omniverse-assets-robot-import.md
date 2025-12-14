---
id: lesson-02-omniverse-assets-robot-import
title: "Lesson 2: Omniverse Assets and Robot Import"
sidebar_position: 2
description: Learn to import and configure existing robot models and 3D assets into Isaac Sim for custom simulation scenarios.
---

# Lesson 2: Omniverse Assets and Robot Import

## Lesson Objectives
- Understand the USD (Universal Scene Description) format and its importance in Omniverse.
- Learn to import existing 3D models and environments from the Omniverse Asset Library.
- Import and configure URDF (Unified Robot Description Format) robot models into Isaac Sim.
- Understand how to modify robot properties and add joints in the simulation environment.

## Prerequisites
- Completion of Lesson 1: Introduction to Isaac Sim and Setup.
- Basic familiarity with 3D modeling concepts.
- Knowledge of URDF structure (from Module 1, if applicable).

## Concept Explanation
NVIDIA Omniverse uses USD as its core scene description format, allowing for interoperability between different 3D applications and real-time collaboration. This lesson focuses on populating your Isaac Sim environments with existing assets—from simple objects to complex robot models—either from Omniverse's vast library or by importing your own. Correctly importing and configuring robot models (often in URDF format) is fundamental to setting up any custom robotics simulation.

## Step-by-Step Technical Breakdown

1.  **USD Basics**: Familiarize yourself with how USD stages are structured, including prims, properties, and layers.
2.  **Omniverse Asset Library**: Browse and drag-and-drop assets (e.g., tables, chairs, props) from the built-in library into your scene.
3.  **URDF Import**:
    *   Use the "File -> Import -> URDF" menu option in Isaac Sim.
    *   Navigate to a URDF file (e.g., from a ROS 2 robot package).
    *   Configure import options, including material, stage units, and fix base link.
4.  **Robot Configuration**: After import, inspect the robot's properties in the Property window. Adjust joint drives, add collision properties, or attach sensors if not defined in the URDF.

## Real-World Analogy
Imagine setting up a play. You need to bring in the stage props (Omniverse assets) and then dress your actors (robot models) in their costumes (URDF configuration), making sure they can move correctly according to the script (simulation physics).

## Hands-On Tasks
1.  Launch Isaac Sim and create a new empty stage.
2.  Open the Omniverse Asset Library and add a few environment props (e.g., a desk, a shelf) to your scene.
3.  Download a simple robot's URDF file (e.g., a TurtleBot3 or a basic manipulator arm) if you don't have one readily available.
4.  Import the URDF file into your Isaac Sim stage.
5.  Run the simulation. If the robot collapses, debug its physics properties (mass, inertia) and joint limits.
6.  (Optional) Try modifying a joint's limits or damping property and observe the change in behavior.

<h2>Python + ROS2 Examples (Conceptual Isaac Sim Python API for URDF Import)</h2>

```python
# This is a conceptual example using Isaac Sim Python API for URDF import.
# Run within Isaac Sim's scripting environment.

from omni.isaac.core import World
from omni.isaac.urdf import _urdf
import carb
import os

# Define the path to your URDF file (replace with your actual path)
# This URDF needs to be accessible from within the Isaac Sim environment.
URDF_PATH = "/path/to/your/robot_description/urdf/my_robot.urdf"

def import_urdf_robot():
    if not os.path.exists(URDF_PATH):
        carb.log_error(f"URDF file not found: {URDF_PATH}")
        return

    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    # Import URDF using the low-level API
    _urdf.create_urdf_robot(
        stage=world.scene.stage,
        urdf_path=URDF_PATH,
        name="my_imported_robot",
        position=carb.Float3(0.0, 0.0, 0.0),
        orientation=carb.Float3(0.0, 0.0, 0.0, 1.0),
        import_config= _urdf.ImportConfig(
            fix_base=False, # True if robot is fixed to ground, False for mobile base
            make_instanceable=False
        )
    )

    world.reset()
    carb.log_info("URDF Robot imported and simulation reset.")
    
    # You can now step the simulation or add more logic
    for _ in range(100):
        world.step(render=True)

# Call the function
import_urdf_robot()
```

## Debugging Tips
-   **URDF Errors**: Invalid URDF files can cause import failures. Use URDF validation tools (e.g., `check_urdf` in ROS) to verify your file.
-   **Asset Paths**: Ensure that all meshes and textures referenced in your USD or URDF files are correctly linked and accessible within Isaac Sim.
-   **Physics Instability**: If your robot behaves erratically or falls apart, check:
    *   Mass and inertia properties in the URDF/USD.
    *   Joint limits and stiffness/damping.
    *   Collision geometries.
-   **Frame Conventions**: Pay attention to coordinate frame conventions (e.g., Z-up vs. Y-up) to avoid unexpected rotations.

## Mini Quiz (4-6 questions)
1.  What is USD, and why is it important for NVIDIA Omniverse and Isaac Sim?
2.  How do you typically import a robot model defined in URDF into Isaac Sim?
3.  Name two parameters you might configure for a robot after importing it into Isaac Sim.
4.  What is the primary benefit of using the Omniverse Asset Library?
5.  What can cause a simulated robot to behave unstably after being imported?

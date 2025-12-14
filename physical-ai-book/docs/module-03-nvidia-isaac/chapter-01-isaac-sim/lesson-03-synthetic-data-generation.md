---
id: lesson-03-synthetic-data-generation
title: "Lesson 3: Synthetic Data Generation"
sidebar_position: 3
description: Leverage Isaac Sim's capabilities to generate diverse and annotated synthetic data for training AI perception models.
---

# Lesson 3: Synthetic Data Generation

## Lesson Objectives
- Understand the benefits of synthetic data for AI model training in robotics.
- Learn to configure various sensors in Isaac Sim for data collection.
- Implement domain randomization techniques to enhance model robustness.
- Generate annotated image datasets (RGB, depth, segmentation, bounding boxes) from Isaac Sim.

## Prerequisites
- Completion of Lesson 1 and 2 of this chapter.
- Basic understanding of machine learning and computer vision datasets.
- Familiarity with Python scripting.

## Concept Explanation
High-quality, annotated data is the lifeblood of deep learning models, but collecting and labeling real-world robotics data is a tedious, expensive, and often dangerous process. Synthetic data generation in Isaac Sim provides an elegant solution. By simulating diverse environments, lighting conditions, and object variations, we can automatically generate large volumes of perfectly labeled data (RGB, depth, instance segmentation, bounding boxes) to train perception models, significantly reducing development time and cost while improving model robustness.

<h2>Step-by-Step Technical Breakdown</h2>

1.  **Sensor Configuration**: Attach and configure various sensors to your robot or environment in Isaac Sim, such as RGB cameras, depth cameras, LiDAR, and IMUs.
2.  **Ground Truth Data Access**: Utilize Isaac Sim's API to access ground truth information directly from the simulation, including:
    *   **RGB**: Standard camera images.
    *   **Depth**: Per-pixel distance to objects.
    *   **Instance Segmentation**: Pixel-wise labels for each unique object instance.
    *   **Bounding Boxes**: 2D or 3D bounding boxes around objects.
3.  **Domain Randomization**: Implement scripting to randomize various aspects of the simulation:
    *   **Textures and Materials**: Randomly change the appearance of objects and surfaces.
    *   **Lighting**: Vary light sources, intensity, and colors.
    *   **Object Poses**: Randomly place objects within a scene.
    *   **Camera Parameters**: Adjust intrinsic and extrinsic camera parameters.
4.  **Data Export**: Write Python scripts to collect and export the generated synthetic data in a format suitable for machine learning frameworks (e.g., COCO, YOLO).

## Real-World Analogy
Instead of hiring a photographer, models, and makeup artists for every product shot, imagine you have a magical 3D printer that can instantly create infinite variations of your product in any setting, with perfect lighting and a label telling you exactly what it is. That's synthetic data generation for AI models.

## Hands-On Tasks
1.  Load a robot and a simple environment (e.g., a table with objects) into Isaac Sim.
2.  Attach an RGB camera to your robot and visualize its output in the viewport.
3.  Write a Python script within Isaac Sim to capture RGB images, depth maps, and instance segmentation masks of objects on the table.
4.  Implement a simple domain randomization script that randomly changes the color/texture of one object every few simulation steps.
5.  Export a small dataset of synthetic images with their corresponding ground truth annotations.

<h2>Python + ROS2 Examples (Conceptual Isaac Sim Python API for Synthetic Data)</h2>

```python
# This is a conceptual example using Isaac Sim Python API for synthetic data generation.
# Run within Isaac Sim's scripting environment.

from omni.isaac.core import World
from omni.isaac.synthetic_utils import SyntheticData
import omni.isaac.sensor as _sensor
import carb
import numpy as np
import random
import os

# Create a folder to save the synthetic data
OUTPUT_DIR = "synthetic_dataset"
os.makedirs(OUTPUT_DIR, exist_ok=True)

def generate_synthetic_data():
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    # Add a simple cube to the scene
    cube_prim = world.scene.add_default_cube(
        prim_path="/World/Cube",
        position=np.array([0.5, 0.0, 0.5]),
        scale=np.array([0.1, 0.1, 0.1])
    )

    # Add a camera to capture data
    camera_prim_path = "/World/Camera"
    camera = _sensor.Camera(
        prim_path=camera_prim_path,
        position=np.array([0.0, 0.0, 1.5]),
        look_at_target=np.array([0.5, 0.0, 0.5]),
        fov_y=60.0
    )
    # This requires the "omni.syntheticdata" extension to be enabled in Isaac Sim
    sd_interface = SyntheticData()

    world.reset()

    # Domain randomization: randomize cube color
    def randomize_cube_color():
        red = random.uniform(0.0, 1.0)
        green = random.uniform(0.0, 1.0)
        blue = random.uniform(0.0, 1.0)
        cube_prim.get_applied_action_graph().get_body_view().set_color(
            np.array([red, green, blue]),
            indices=[0] # Apply to the first (only) cube
        )

    for i in range(20): # Generate 20 samples
        world.step(render=True)
        randomize_cube_color() # Apply randomization
        world.step(render=True) # Step again to render new color

        # Capture RGB, Depth, and Instance Segmentation
        rgb_data = sd_interface.get_image_data(camera_prim_path, "rgba")
        depth_data = sd_interface.get_image_data(camera_prim_path, "depth")
        segmentation_data = sd_interface.get_image_data(camera_prim_path, "instanceSegmentation")

        # Save data (conceptual - in real code, you'd process/save numpy arrays)
        # For actual image saving, you'd use PIL or OpenCV
        carb.log_info(f"Captured data for frame {i}: RGB shape {rgb_data.shape}, Depth shape {depth_data.shape}")
        # Example of saving:
        # np.save(os.path.join(OUTPUT_DIR, f"rgb_{i}.npy"), rgb_data)
        # np.save(os.path.join(OUTPUT_DIR, f"depth_{i}.npy"), depth_data)
        # np.save(os.path.join(OUTPUT_DIR, f"seg_{i}.npy"), segmentation_data)

    carb.log_info(f"Synthetic data generation finished. Data saved to {OUTPUT_DIR}")

# Call the function
generate_synthetic_data()
```

## Debugging Tips
-   **Extension Activation**: Ensure the `omni.syntheticdata` extension (or similar data generation extensions) is enabled in Isaac Sim's Extension Manager.
-   **Sensor Placement**: Verify that your cameras are placed correctly and have an unobstructed view of the objects you want to capture.
-   **Annotation Format**: Double-check that your exported data format matches the requirements of your target machine learning framework.
-   **Domain Randomization Bugs**: Randomization scripts can sometimes introduce undesirable or unrealistic scene configurations. Test randomization thoroughly.

## Mini Quiz (4-6 questions)
1.  What is synthetic data, and why is it beneficial for training AI models in robotics?
2.  Name three types of ground truth annotations that can be generated in Isaac Sim.
3.  Explain the concept of domain randomization and its purpose.
4.  How do you access ground truth data from Isaac Sim's API?
5.  What are some limitations of relying solely on synthetic data for AI model training?

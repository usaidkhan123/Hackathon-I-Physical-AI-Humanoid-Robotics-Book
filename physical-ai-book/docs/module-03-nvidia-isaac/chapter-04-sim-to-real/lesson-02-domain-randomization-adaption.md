---
id: lesson-02-domain-randomization-adaption
title: "Lesson 2: Domain Randomization and Adaption"
sidebar_position: 2
description: Apply domain randomization techniques in Isaac Sim to make AI models more robust to real-world variations and reduce the sim-to-real gap.
---

# Lesson 2: Domain Randomization and Adaption

## Lesson Objectives
- Understand the principles of domain randomization for sim-to-real transfer.
- Learn to implement various randomization techniques in Isaac Sim.
- Apply strategies for adapting models trained in simulation to perform well on physical robots.
- Evaluate the effectiveness of domain randomization on AI model performance.

## Prerequisites
- Completion of Lesson 1: Understanding the Sim-to-Real Gap.
- Basic understanding of machine learning model training and overfitting.
- Familiarity with Isaac Sim Python scripting.

## Concept Explanation
Domain randomization is a powerful technique to bridge the sim-to-real gap. Instead of trying to create a perfect simulation that exactly matches reality, domain randomization intentionally varies numerous aspects of the simulation (e.g., textures, lighting, object positions, sensor noise, physics parameters) during training. This forces the AI model to learn features that are invariant to these variations, making it more robust and transferable to the diverse and unpredictable conditions of the real world. Coupled with other adaptation strategies, it significantly improves real-world performance.

## Step-by-Step Technical Breakdown

1.  **Randomization Parameters**: Identify key parameters in your simulation environment that can be randomized and directly impact your AI model's perception or control. These often include:
    *   **Visual**: Textures, colors, lighting conditions, background elements.
    *   **Physical**: Object masses, friction coefficients, joint damping.
    *   **Sensor**: Noise models, sensor offsets, camera intrinsic parameters.
2.  **Implementing Randomization in Isaac Sim**: Use the Isaac Sim Python API to programmatically randomize these parameters at the start of each training episode or at regular intervals.
3.  **Model Training**: Train your AI model (e.g., an object detector or a reinforcement learning policy) using the diverse synthetic data generated from the randomized simulation.
4.  **Sim-to-Real Adaptation**: Post-training, apply strategies like fine-tuning with a small amount of real-world data, or employing domain adaptation techniques to further improve performance on physical hardware.

## Real-World Analogy
Imagine a driving instructor who trains students by putting them in hundreds of different cars, on countless roads, in all weather conditions, at various times of day. By the time they get to a real car on a real road, they've seen it all and can adapt easily. Domain randomization does this for AI models: it shows them so many variations in simulation that they become experts at generalizing to the real world.

## Hands-On Tasks
1.  Load an Isaac Sim scene with a robot and some objects.
2.  Write a Python script to:
    *   Randomly change the color or texture of an object at the beginning of each simulation reset.
    *   Randomly adjust the intensity or color of a light source.
3.  Integrate these randomization scripts into a loop that generates a large dataset of diverse images.
4.  (Conceptual) If you have a simple object detection model, briefly discuss how you would train it on this randomized synthetic data and what benefits you would expect.

<h2>Python + ROS2 Examples (Conceptual Isaac Sim Domain Randomization Script)</h2>

```python
# This is a conceptual example of a Python script for domain randomization in Isaac Sim.
# Run within Isaac Sim's scripting environment.

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import create_prim, get_prim_at_path
from omni.isaac.core.utils.stage import create_new_stage
from pxr import Gf, UsdLux, Sdf
import carb
import numpy as np
import random
import os

def setup_scene_and_randomize():
    create_new_stage()
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    # Add a dynamic cube object
    cube = world.scene.add(
        DynamicCuboid(
            prim_path="/World/cube",
            position=np.array([0.0, 0.0, 0.5]),
            scale=np.array([0.2, 0.2, 0.2]),
            color=np.array([0.5, 0.5, 0.5]),
            name="dynamic_cube"
        )
    )

    # Add a dome light for global illumination
    create_prim("/World/DomeLight", "DomeLight")
    dome_light_prim = get_prim_at_path("/World/DomeLight")
    dome_light = UsdLux.DomeLight(dome_light_prim)

    world.reset()
    carb.log_info("Scene setup complete. Starting randomization.")

    # --- Domain Randomization Loop ---
    for episode in range(10):
        carb.log_info(f"--- Episode {episode + 1}: Applying Randomization ---")
        
        # 1. Randomize Cube Color
        random_color = np.random.rand(3)
        cube.set_color(random_color)
        carb.log_info(f"  Cube color randomized to: {random_color}")

        # 2. Randomize Light Intensity
        random_intensity = random.uniform(1000.0, 5000.0)
        dome_light.GetIntensityAttr().Set(random_intensity)
        carb.log_info(f"  Light intensity randomized to: {random_intensity}")

        # 3. Randomize Cube Position (slightly)
        random_offset_x = random.uniform(-0.1, 0.1)
        random_offset_y = random.uniform(-0.1, 0.1)
        cube.set_world_pose(position=np.array([random_offset_x, random_offset_y, 0.5]))
        carb.log_info(f"  Cube position randomized to: ({random_offset_x:.2f}, {random_offset_y:.2f})")

        # 4. Randomize Texture (conceptual - requires more complex USD manipulation)
        # You would typically have a pool of textures and randomly assign one to the object
        # Example (conceptual):
        # random_texture_path = get_assets_root_path() + "/NvProps/Materials/Wood/wood_cherry_red.usd"
        # prim_material = UsdShade.Material.Define(world.stage, Sdf.Path("/World/Material_Random"))
        # prim_material.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(random_color)
        # UsdShade.MaterialBindingAPI(cube.prim).Bind(prim_material)

        world.step(render=True) # Step to apply changes

        # Now, if this were a training loop, you would capture synthetic data here
        # (e.g., camera images, depth, segmentation)
        # Example: sd_interface.get_image_data(...)

        # Reset simulation for next episode
        world.reset()
        world.step(render=True) # Allow reset to settle

    carb.log_info("Domain randomization demonstration finished.")

# Call the function
setup_scene_and_randomize()
```

## Debugging Tips
-   **Granularity of Randomization**: Start with simple randomizations and gradually increase complexity. Over-randomization can make a scene unrealistic or untrainable.
-   **Visualization**: Periodically visualize your randomized scenes to ensure they are plausible and contain the desired variations.
-   **Model Performance**: Track the performance of your AI model on both synthetic (randomized) data and real-world data to evaluate the effectiveness of domain randomization.
-   **Compute Resources**: Domain randomization can increase the computational requirements for data generation. Optimize your randomization ranges and sensor settings.

## Mini Quiz (4-6 questions)
1.  What is the main goal of domain randomization?
2.  How does domain randomization help bridge the sim-to-real gap?
3.  Name three types of parameters that can be randomized in Isaac Sim for this technique.
4.  Why is it better to randomize parameters than to try and perfectly match reality in simulation?
5.  What happens to an AI model if it's only trained on data from a non-randomized, perfectly clean simulation?
```
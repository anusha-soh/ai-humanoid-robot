# Chapter 9: Introduction to NVIDIA Isaac Sim

## Overview

Welcome to NVIDIA Isaac Sim, the next-generation simulation environment designed for AI robotics development. Unlike traditional physics simulators like Gazebo, Isaac Sim leverages NVIDIA's RTX technology to provide photorealistic rendering capabilities that can generate synthetic training data for computer vision and perception systems. This chapter will introduce you to Isaac Sim's architecture, its integration with ROS 2, and how to create realistic simulation environments for AI training.

Isaac Sim represents a paradigm shift in robotics simulation, moving from physics-focused simulation to perception-focused simulation. While Gazebo excels at physics accuracy, Isaac Sim excels at visual realism, making it ideal for training perception algorithms that need to operate in real-world conditions. You'll learn how to set up Isaac Sim, import your robot models, and create visually stunning environments that can be used for perception training. By the end of this chapter, you'll have created a photorealistic warehouse environment with your humanoid robot.

### Learning Objectives

By the end of this chapter, you will be able to:
- Compare and contrast Isaac Sim with traditional simulators like Gazebo
- Set up and configure Isaac Sim for robotics development
- Import and configure robot models in Isaac Sim
- Create photorealistic environments with RTX rendering
- Generate synthetic training data for computer vision applications

### Prerequisites

Before starting this chapter, you should have:
- Completed Modules 1 and 2 (ROS 2 basics and simulation)
- Understanding of robot description formats (URDF/SDF)
- Access to an NVIDIA GPU with RTX capabilities (or use fallbacks)
- Basic familiarity with computer vision concepts

## Concepts

### Isaac Sim vs Gazebo

While both Isaac Sim and Gazebo are powerful simulation environments, they serve different purposes:

- **Isaac Sim**: Focuses on photorealistic rendering and perception training
  - RTX ray-traced lighting and materials
  - High-fidelity sensor simulation
  - Synthetic data generation capabilities
  - NVIDIA-specific optimizations

- **Gazebo**: Focuses on physics accuracy and general simulation
  - Realistic physics simulation with multiple engines
  - Broad sensor support
  - Cross-platform compatibility
  - Established robotics ecosystem

### Isaac Sim Architecture

Isaac Sim is built on NVIDIA Omniverse, a simulation and collaboration platform:

- **Omniverse Nucleus**: Central server for asset management and collaboration
- **Kit Framework**: Extensible application framework
- **USD (Universal Scene Description)**: Scene representation format
- **RTX Renderer**: Physically-based rendering engine
- **PhysX**: NVIDIA's physics engine

### RTX Rendering Capabilities

Isaac Sim's rendering pipeline includes:

- **Ray Tracing**: Realistic light transport and global illumination
- **Material Definition Language (MDL)**: Physically-based material definitions
- **Volume Rendering**: Fog, smoke, and atmospheric effects
- **Post-Processing**: Color grading, depth of field, motion blur

### Synthetic Data Generation

Isaac Sim provides tools for generating training data:

- **Domain Randomization**: Varying visual properties for robust training
- **Sensor Simulation**: Camera, LiDAR, and other sensor models
- **Ground Truth Annotation**: Automatic labeling of objects and properties
- **Data Export**: Formats compatible with popular ML frameworks

### Isaac ROS Integration

Isaac Sim connects to ROS 2 through Isaac ROS:

- **Isaac ROS GEMs**: GPU-accelerated perception modules
- **ROS Bridge**: Standard ROS 2 communication
- **Sensor Plugins**: ROS-compatible sensor interfaces
- **Robot Bridge**: Joint state and control interfaces

## Examples

### Example 1: Setting up Isaac Sim Environment

First, let's create a basic Isaac Sim environment with our humanoid robot:

```python
#!/usr/bin/env python3
# This example demonstrates how to set up a basic Isaac Sim environment

import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.utils.carb import wait_stage_update_end
import carb

# Initialize Isaac Sim
omni.kit.pipapi.pip_install("numpy")

# Create a world instance
my_world = World(stage_units_in_meters=1.0)

# Add a ground plane
my_world.scene.add_default_ground_plane()

# Add a simple warehouse environment
def create_warehouse():
    # Create warehouse walls
    create_prim(
        prim_path="/World/Wall_Left",
        prim_type="Cuboid",
        position=[-5.0, 0, 2.0],
        scale=[0.2, 10.0, 4.0],
        attributes={"color": [0.8, 0.8, 0.8]}
    )

    create_prim(
        prim_path="/World/Wall_Right",
        prim_type="Cuboid",
        position=[5.0, 0, 2.0],
        scale=[0.2, 10.0, 4.0],
        attributes={"color": [0.8, 0.8, 0.8]}
    )

    create_prim(
        prim_path="/World/Wall_Back",
        prim_type="Cuboid",
        position=[0, -5.0, 2.0],
        scale=[10.0, 0.2, 4.0],
        attributes={"color": [0.8, 0.8, 0.8]}
    )

    # Add warehouse shelves
    for i in range(3):
        for j in range(2):
            create_prim(
                prim_path=f"/World/Shelf_{i}_{j}",
                prim_type="Cuboid",
                position=[-3.0 + i*3.0, 3.0, 0.5 + j*1.0],
                scale=[1.5, 0.1, 0.8],
                attributes={"color": [0.5, 0.3, 0.1]}
            )

# Create the warehouse
create_warehouse()

# Add lighting
from omni.isaac.core.utils.prims import get_prim_at_path
from pxr import UsdLux

# Add a dome light for environment lighting
dome_light = create_prim(
    prim_path="/World/DomeLight",
    prim_type="DomeLight",
    attributes={"color": [0.8, 0.8, 0.8], "intensity": 300}
)

# Add a distant light for directional lighting
distant_light = create_prim(
    prim_path="/World/DistantLight",
    prim_type="DistantLight",
    position=[0, 0, 10],
    attributes={"color": [0.9, 0.9, 0.9], "intensity": 1000}
)

print("Warehouse environment created successfully")
```

### Example 2: Importing Robot Model into Isaac Sim

Now let's import our robot model into Isaac Sim:

```python
#!/usr/bin/env python3
# Example of importing a robot model into Isaac Sim

from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.prims import define_prim
import numpy as np

# Create world instance
my_world = World(stage_units_in_meters=1.0)

# Add ground plane
my_world.scene.add_default_ground_plane()

# Define a simple humanoid robot using prims (for demonstration)
def create_simple_humanoid():
    # Create base link
    base_prim = define_prim("/World/Humanoid/base_link", "Cylinder")
    base_prim.GetAttribute("radius").Set(0.15)
    base_prim.GetAttribute("height").Set(0.2)

    # Create torso
    torso_prim = define_prim("/World/Humanoid/torso", "Cone")
    torso_prim.GetAttribute("radius").Set(0.1)
    torso_prim.GetAttribute("height").Set(0.5)

    # Position torso relative to base
    torso_prim.GetAttribute("xformOp:translate").Set((0, 0, 0.25))

    # Create head
    head_prim = define_prim("/World/Humanoid/head", "Sphere")
    head_prim.GetAttribute("radius").Set(0.1)
    head_prim.GetAttribute("xformOp:translate").Set((0, 0, 0.6))

    # Create arms
    left_arm_prim = define_prim("/World/Humanoid/left_arm", "Cylinder")
    left_arm_prim.GetAttribute("radius").Set(0.03)
    left_arm_prim.GetAttribute("height").Set(0.4)
    left_arm_prim.GetAttribute("xformOp:translate").Set((0.15, 0, 0.4))

    right_arm_prim = define_prim("/World/Humanoid/right_arm", "Cylinder")
    right_arm_prim.GetAttribute("radius").Set(0.03)
    right_arm_prim.GetAttribute("height").Set(0.4)
    right_arm_prim.GetAttribute("xformOp:translate").Set((-0.15, 0, 0.4))

# Create the simple humanoid
create_simple_humanoid()

print("Simple humanoid robot created in Isaac Sim")
```

### Example 3: Configuring RTX Rendering Settings

Let's configure the RTX rendering settings for photorealistic output:

```python
#!/usr/bin/env python3
# Configure RTX rendering settings for Isaac Sim

import carb
from omni import kit
from omni.kit.viewport.utility import get_active_viewport

def configure_rtx_settings():
    """Configure RTX rendering settings for photorealistic output"""

    # Enable RTX rendering
    settings = carb.settings.get_settings()
    settings.set("/rtx/legacyAmbientLight", False)
    settings.set("/rtx/ambientLight", True)

    # Set rendering quality to maximum
    settings.set("/rtx/quality/preference", 3)  # Maximum quality
    settings.set("/rtx/quality/active", True)

    # Enable global illumination
    settings.set("/rtx/rendermode", "RaytracedLightmap")
    settings.set("/rtx/raytracing/enable", True)

    # Enable denoisers for faster rendering
    settings.set("/rtx/raytracing/denoise/enable", True)
    settings.set("/rtx/raytracing/denoise/enableDlss", True)

    # Enable volumetric effects
    settings.set("/rtx/volumetric/scattering/enable", True)

    # Configure post-processing
    settings.set("/rtx/post/dlss/enable", True)
    settings.set("/rtx/post/dlss/sharpness", 0.5)

    # Enable motion blur
    settings.set("/rtx/post/motionblur/enable", True)
    settings.set("/rtx/post/motionblur/quality", 2)  # High quality

    print("RTX rendering settings configured for photorealistic output")

# Apply RTX settings
configure_rtx_settings()
```

### Example 4: Sensor Simulation and Data Generation

Create a script to simulate sensors and generate synthetic data:

```python
#!/usr/bin/env python3
# Example of sensor simulation and synthetic data generation in Isaac Sim

import omni
from omni.isaac.core import World
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.sensor import Camera
from omni.isaac.core.utils.viewports import set_camera_view
import numpy as np
import cv2
import os
from PIL import Image

# Create world instance
my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

# Add a camera sensor
camera = Camera(
    prim_path="/World/Camera",
    position=np.array([1.5, 1.5, 1.0]),
    orientation=np.array([0.707, 0, 0, 0.707])  # Rotate to look at origin
)

# Add the camera to the scene
my_world.scene.add(camera)

# Function to capture and save synthetic images
def capture_synthetic_data():
    """Capture synthetic images with different lighting conditions"""

    # Create directory for synthetic data
    data_dir = "synthetic_data"
    os.makedirs(data_dir, exist_ok=True)

    # Get camera properties
    camera.initialize()

    # Simulate different lighting conditions
    lighting_conditions = [
        {"intensity": 500, "color": [0.9, 0.9, 0.9], "name": "bright"},
        {"intensity": 200, "color": [0.8, 0.7, 0.6], "name": "warm"},
        {"intensity": 800, "color": [0.5, 0.6, 0.9], "name": "cool"}
    ]

    for i, lighting in enumerate(lighting_conditions):
        # Update lighting
        light_prim = get_prim_at_path("/World/DistantLight")
        if light_prim:
            light_prim.GetAttribute("inputs:intensity").Set(lighting["intensity"])
            light_prim.GetAttribute("inputs:color").Set(lighting["color"])

        # Step the world to update lighting
        my_world.step(render=True)

        # Capture RGB image
        rgb_image = camera.get_rgb()
        if rgb_image is not None:
            # Convert to PIL Image and save
            pil_image = Image.fromarray(rgb_image, mode="RGB")
            filename = f"{data_dir}/synthetic_image_{lighting['name']}_{i:03d}.png"
            pil_image.save(filename)
            print(f"Saved synthetic image: {filename}")

    print(f"Synthetic data generation completed. Images saved to {data_dir}")

# Run the synthetic data generation
capture_synthetic_data()
```

## Exercise

### Required Exercise: Isaac Sim Warehouse Environment

Create a complete Isaac Sim environment with perception capabilities:

**Task**: Build an environment with:
1. A warehouse scene with realistic objects and lighting
2. Your humanoid robot imported and configured
3. A camera sensor that captures synthetic training data
4. Different lighting conditions for domain randomization

**Acceptance Criteria**:
- Warehouse scene renders with RTX photorealistic quality
- Robot moves and responds to commands appropriately
- Camera captures realistic images suitable for ML training
- Environment can be varied for synthetic data generation

**Implementation Steps**:
1. Create a warehouse environment with shelves, boxes, and obstacles
2. Import your humanoid robot model into Isaac Sim
3. Configure RTX rendering settings for photorealistic output
4. Implement sensor simulation with camera capture
5. Add domain randomization capabilities for lighting and objects
6. Test the complete simulation system

**Gazebo Fallback**: If you don't have access to an NVIDIA GPU, implement the same functionality using Gazebo with the following adjustments:
- Use Gazebo's OGRE rendering instead of RTX
- Use standard Gazebo sensors instead of Isaac Sim-specific sensors
- Generate synthetic data using Gazebo's sensor capabilities

**Extension Challenges**:
- **Beginner**: Add more objects and furniture to the warehouse environment
- **Intermediate**: Implement LiDAR sensor simulation in Isaac Sim
- **Advanced**: Create a complete synthetic dataset generation pipeline with automatic annotations

## References

- [NVIDIA Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/isaacsim_overview.html)
- [Isaac Sim Tutorials](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_intro.html)
- [Omniverse USD Guide](https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/prim_creation.html)
- [RTX Rendering in Isaac Sim](https://docs.omniverse.nvidia.com/isaacsim/latest/features/rendering/index.html)
- [Synthetic Data Generation](https://developer.nvidia.com/blog/generating-synthetic-training-data-for-object-detection-using-isaac-sim/)
- [Isaac ROS Integration](https://github.com/NVIDIA-ISAAC-ROS)

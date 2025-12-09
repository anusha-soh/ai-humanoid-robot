# Chapter 3: Describing Robots - Introduction to URDF

## Overview

In the previous chapters, you learned how nodes communicate through topics and services. Now it's time to understand how robots are described in ROS 2. The Unified Robot Description Format (URDF) is the standard way to represent robot models in ROS, defining everything from the physical structure to the kinematic relationships between parts.

URDF is crucial for robotics because it allows you to describe your robot's physical properties, which enables simulation, visualization, motion planning, and control. You'll learn how to create robot descriptions using links (rigid bodies) and joints (connections), understand coordinate frame transformations with TF2, and visualize your robot in RViz2. By the end of this chapter, you'll have created a complete humanoid robot model with articulated arms.

### Learning Objectives

By the end of this chapter, you will be able to:
- Explain the purpose and structure of URDF files
- Define robot links with visual, collision, and inertial properties
- Create joints to connect links in kinematic chains
- Use TF2 to understand coordinate frame transformations
- Visualize robot models in RViz2

### Prerequisites

Before starting this chapter, you should have:
- Completed Chapters 1 and 2 (ROS 2 basics and communication)
- Understanding of basic geometry and coordinate systems
- Familiarity with XML syntax (URDF is XML-based)
- Basic knowledge of physics concepts (mass, center of mass, inertia)

## Concepts

### What is URDF?

URDF (Unified Robot Description Format) is an XML-based format that describes robot models in ROS. It defines the robot's physical structure, including:

- **Links**: Rigid bodies that make up the robot (e.g., arms, torso, wheels)
- **Joints**: Connections between links (e.g., revolute, prismatic, fixed)
- **Visual properties**: How the robot looks in simulation and visualization
- **Collision properties**: How the robot interacts with the environment in simulation
- **Inertial properties**: Physical properties needed for simulation (mass, center of mass, inertia)

### Links: The Building Blocks of Robot Structure

A **link** in URDF represents a rigid body part of the robot. Each link can have:

- **Visual**: How the link appears in visualization (shape, color, material)
- **Collision**: How the link interacts with other objects in simulation
- **Inertial**: Physical properties needed for physics simulation (mass, center of mass, inertia matrix)

### Joints: Connecting the Links

A **joint** connects two links and defines how they can move relative to each other. URDF supports several joint types:

- **Fixed**: No movement between links (rigid connection)
- **Revolute**: Single-axis rotation (like a hinge)
- **Continuous**: Unlimited rotation around a single axis
- **Prismatic**: Linear sliding motion along one axis
- **Floating**: 6 degrees of freedom (rarely used)
- **Planar**: Motion constrained to a plane

### Kinematic Chains

Robot arms and other articulated mechanisms are represented as **kinematic chains** - sequences of links connected by joints. The chain starts from a base link and extends to end-effectors (like hands or tools).

### Coordinate Frames and TF2

The **Transform Library (TF2)** manages coordinate frame transformations in ROS. Each link has its own coordinate frame, and TF2 keeps track of how these frames relate to each other over time. This allows:

- Transforming points between different coordinate frames
- Understanding the pose of robot parts relative to each other
- Visualizing robot motion in RViz2

### Robot State Publisher

The **robot_state_publisher** is a ROS node that reads a URDF model and publishes the state of the joints to TF2. This enables visualization and simulation of the robot's current configuration.

## Examples

### Example 1: Creating a Simple 2-Link Robot Arm

Let's start with a simple 2-link robot arm to understand the basics:

```xml
<?xml version="1.0"?>
<robot name="simple_arm">
  <!-- Base link (fixed) -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- First arm link -->
  <link name="arm_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.5"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint connecting base to arm -->
  <joint name="base_to_arm" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
</robot>
```

### Example 2: Creating a Humanoid Torso

Now let's create a basic humanoid torso with a head, neck joint, and shoulders:

```xml
<?xml version="1.0"?>
<robot name="humanoid_torso">
  <!-- Base link: pelvis -->
  <link name="pelvis">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Spine -->
  <link name="spine">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint connecting pelvis to spine -->
  <joint name="pelvis_to_spine" type="fixed">
    <parent link="pelvis"/>
    <child link="spine"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Neck joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="spine"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="5" velocity="1"/>
  </joint>

  <!-- Left shoulder -->
  <link name="left_shoulder">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.05"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Left shoulder joint -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="spine"/>
    <child link="left_shoulder"/>
    <origin xyz="0.1 0 0.2" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
</robot>
```

### Example 3: Launching Robot Visualization

Create a launch file to visualize the robot in RViz2:

```python
#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get URDF file path
    urdf_file = os.path.join(
        get_package_share_directory('robot_description'),
        'urdf',
        'humanoid_torso.urdf'
    )

    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Robot State Publisher node
    params = {'robot_description': robot_desc}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # RViz2 node
    rviz_config = os.path.join(
        get_package_share_directory('robot_description'),
        'rviz',
        'robot_visualization.rviz'
    )
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_rviz
    ])
```

### Example 4: Creating a Complete Humanoid with Arms

Now let's extend our robot to include full arms:

```xml
<?xml version="1.0"?>
<robot name="humanoid_with_arms">
  <!-- Include the torso from previous example -->
  <link name="pelvis">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <link name="spine">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="pelvis_to_spine" type="fixed">
    <parent link="pelvis"/>
    <child link="spine"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="spine"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="5" velocity="1"/>
  </joint>

  <!-- Left arm -->
  <link name="left_shoulder">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.05"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="spine"/>
    <child link="left_shoulder"/>
    <origin xyz="0.1 0 0.2" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_shoulder"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.05 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.36" upper="0" effort="10" velocity="1"/>
  </joint>

  <link name="left_forearm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.7"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_forearm_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_forearm"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="0" effort="5" velocity="1"/>
  </joint>

  <!-- Right arm (mirror of left) -->
  <link name="right_shoulder">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.05"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_shoulder_joint" type="revolute">
    <parent link="spine"/>
    <child link="right_shoulder"/>
    <origin xyz="-0.1 0 0.2" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_shoulder"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.05 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.36" effort="10" velocity="1"/>
  </joint>

  <link name="right_forearm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.7"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_forearm_joint" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_forearm"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.0" effort="5" velocity="1"/>
  </joint>
</robot>
```

## Exercise

### Required Exercise: Extend the Humanoid Robot

Build upon the basic humanoid model to create a more complete robot with articulated arms:

**Task**: Create an extended humanoid model with:
1. Complete upper body (torso, head, arms with shoulders, elbows, and wrists)
2. Add simple grippers to the hands
3. Create a launch file to visualize the robot in RViz2
4. Add joint state publisher to control the joints interactively

**Acceptance Criteria**:
- Robot model includes at least 12 links and 10 joints
- All links have proper visual, collision, and inertial properties
- Robot displays correctly in RViz2 with proper kinematic chains
- Joint controllers allow interactive manipulation of the robot pose

**Implementation Steps**:
1. Extend the URDF from Example 4 to include hands and simple grippers
2. Create a launch file that starts robot_state_publisher and joint_state_publisher_gui
3. Create an RViz2 configuration file to properly visualize the robot
4. Test the visualization and joint control

**Extension Challenges**:
- **Beginner**: Add colors to different parts of the robot to make it visually distinct
- **Intermediate**: Add a simple gripper mechanism with actuated fingers
- **Advanced**: Implement a controller to make the robot wave its arm autonomously

## References

- [URDF/XML Format Documentation](http://wiki.ros.org/urdf/XML)
- [Robot State Publisher](http://wiki.ros.org/robot_state_publisher)
- [TF2 Overview](http://wiki.ros.org/tf2)
- [RViz User Guide](http://wiki.ros.org/rviz/UserGuide)
- [Creating Your First URDF](http://gazebosim.org/tutorials/?tut=ros_urdf)
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)

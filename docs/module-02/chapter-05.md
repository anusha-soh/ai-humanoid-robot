# Chapter 5: Physics Simulation Fundamentals

## Overview

Welcome to the world of physics simulation! In this chapter, you'll learn how to create realistic virtual environments where robots can be tested safely before deployment in the real world. Physics simulation is a cornerstone of modern robotics development, allowing you to validate control algorithms, test sensor configurations, and experiment with robot designs without the risks and costs associated with physical hardware.

Physics simulation engines like those used in Gazebo calculate the motion of objects by solving complex equations of motion, accounting for forces like gravity, friction, and collisions. You'll learn how to configure these parameters to match real-world physics, set up simulation environments, and integrate your robots with the simulation. By the end of this chapter, you'll have created a box stacking simulation that demonstrates realistic physics interactions.

### Learning Objectives

By the end of this chapter, you will be able to:
- Explain the principles of physics simulation in robotics
- Configure physics parameters (gravity, friction, damping) in Gazebo
- Create simulation environments with realistic physical properties
- Integrate URDF robots with physics simulation
- Validate simulation results against real-world expectations

### Prerequisites

Before starting this chapter, you should have:
- Completed Module 1 (ROS 2 basics and rclpy integration)
- Understanding of URDF robot descriptions from Chapter 3
- Basic knowledge of physics concepts (forces, motion, friction)
- Familiarity with coordinate frames and transformations

## Concepts

### Physics Simulation in Robotics

Physics simulation in robotics involves creating a virtual environment where the laws of physics are accurately modeled. This allows robots to interact with objects and environments in a way that closely mirrors real-world behavior. Key aspects include:

- **Rigid Body Dynamics**: Simulating the motion of solid objects that don't deform
- **Collision Detection**: Determining when objects make contact
- **Contact Physics**: Calculating forces and responses when objects touch
- **Constraints**: Limiting the motion of objects (like joints in robots)

### Gazebo Architecture

Gazebo is built on a plugin architecture that allows for modular functionality:

- **Physics Engine**: Underlying engine (ODE, Bullet, DART) that performs calculations
- **Sensors**: Simulated sensors (cameras, LiDAR, IMU, etc.) that provide realistic data
- **Render Engine**: Graphics rendering for visualization
- **ROS Interface**: Bridge between Gazebo and ROS for control and data exchange

### Physics Parameters

Physics simulation relies on accurate parameters to produce realistic results:

- **Gravity**: Acceleration due to gravitational force (typically -9.8 m/s² on Earth)
- **Friction**: Resistance to motion when surfaces contact (static and dynamic)
- **Damping**: Energy dissipation that slows motion over time
- **Restitution**: "Bounciness" of collisions (0 = no bounce, 1 = perfectly elastic)

### Simulation Fidelity

The accuracy of a physics simulation depends on several factors:

- **Time Step**: Smaller time steps increase accuracy but require more computation
- **Solver Parameters**: Settings that affect how physics equations are solved
- **Model Quality**: Detailed models with accurate mass properties and collision geometry
- **Parameter Tuning**: Adjusting simulation parameters to match real-world behavior

## Examples

### Example 1: Creating a Basic Gazebo World

Let's start by creating a simple world file with basic physics parameters:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="physics_world">
    <!-- Physics engine configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Environment lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Add a simple box object -->
    <model name="box">
      <pose>0 0 0.5 0 0 0</pose>
      <static>false</static>
      <link name="box_link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.08333</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.08333</iyy>
            <iyz>0</iyz>
            <izz>0.08333</izz>
          </inertia>
        </inertial>
        <collision name="box_collision">
          <geometry>
            <box>
              <size>0.2 0.2 0.2</size>
            </box>
          </geometry>
        </collision>
        <visual name="box_visual">
          <geometry>
            <box>
              <size>0.2 0.2 0.2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.8 0.2 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Example 2: Physics Parameters Configuration

Create a launch file to start Gazebo with our world:

```python
#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # World file path
    world_file = os.path.join(
        get_package_share_directory('simulation_examples'),
        'worlds',
        'physics_world.world'
    )

    # Include Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch/gazebo.launch.py'
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true'
        }.items()
    )

    return LaunchDescription([
        gazebo
    ])
```

### Example 3: Creating a Physics-Tuned Robot Model

Now let's create a simple robot model with proper physics properties:

```xml
<?xml version="1.0" ?>
<robot name="physics_test_robot">
  <!-- Base link with proper physics properties -->
  <link name="base_link">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>

    <collision name="base_collision">
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.15"/>
      </geometry>
    </collision>

    <visual name="base_visual">
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.15"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

  <!-- Add a simple arm link -->
  <link name="arm_link">
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>

    <collision name="arm_collision">
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.02"/>
      </geometry>
    </collision>

    <visual name="arm_visual">
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.02"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>

  <!-- Joint connecting base to arm -->
  <joint name="base_to_arm" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
    <dynamics damping="0.1" friction="0.01"/>
  </joint>
</robot>
```

### Example 4: Testing Physics Properties

Create a Python node to interact with physics simulation:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Twist, Point, Pose
import time


class PhysicsTestNode(Node):
    """
    Node to test physics simulation by applying forces to objects.
    Demonstrates interaction with Gazebo physics engine.
    """

    def __init__(self):
        super().__init__('physics_test_node')

        # Create service client to set entity state
        self.set_state_client = self.create_client(
            SetEntityState,
            '/set_entity_state'
        )

        while not self.set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_entity_state service...')

        # Timer to apply forces periodically
        self.timer = self.create_timer(2.0, self.apply_force_callback)
        self.cycle = 0

        self.get_logger().info('Physics test node started')

    def apply_force_callback(self):
        # Create entity state message
        entity_state = EntityState()
        entity_state.name = 'box'  # Name of the object in Gazebo
        entity_state.pose = Pose()
        entity_state.pose.position.x = 0.0
        entity_state.pose.position.y = 0.0
        entity_state.pose.position.z = 0.5
        entity_state.twist = Twist()

        # Apply different forces based on cycle
        if self.cycle % 2 == 0:
            entity_state.twist.linear.x = 0.5  # Push in x direction
            self.get_logger().info('Applying force in x direction')
        else:
            entity_state.twist.linear.y = 0.5  # Push in y direction
            self.get_logger().info('Applying force in y direction')

        # Create service request
        request = SetEntityState.Request()
        request.state = entity_state

        # Call service asynchronously
        future = self.set_state_client.call_async(request)
        future.add_done_callback(self.force_response_callback)

        self.cycle += 1

    def force_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Force applied successfully')
            else:
                self.get_logger().error(f'Failed to apply force: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PhysicsTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Exercise

### Required Exercise: Box Stacking Simulation

Create a physics simulation that demonstrates realistic box stacking behavior:

**Task**: Build a simulation with:
1. Multiple boxes with proper physics properties
2. A ground plane with appropriate friction
3. A robot arm that can stack boxes
4. Physics parameters tuned for realistic behavior

**Acceptance Criteria**:
- Boxes stack stably without unrealistic sliding or bouncing
- Physics simulation runs in real-time (1:1 real-time factor)
- Robot arm can successfully stack boxes without toppling
- Simulation demonstrates proper collision detection and response

**Implementation Steps**:
1. Create a world file with multiple boxes and a ground plane
2. Set appropriate physics parameters (friction, restitution, damping)
3. Create a simple robot arm URDF with proper inertial properties
4. Write a controller to stack boxes systematically
5. Test the simulation and tune parameters for stability

**Extension Challenges**:
- **Beginner**: Add different sized boxes to the simulation
- **Intermediate**: Implement a random box drop simulation with realistic physics
- **Advanced**: Create a physics-based sorting system that separates boxes by size

## References

- [Gazebo Physics Documentation](http://gazebosim.org/tutorials?tut=physics)
- [SDF World Format](http://sdformat.org/spec)
- [Physics Parameter Tuning Guide](http://gazebosim.org/tutorials?tut=physics_params)
- [URDF to SDF Conversion](http://gazebosim.org/tutorials?tut=ros_urdf)
- [ODE Physics Engine](https://www.ode.org/)
- [Bullet Physics Engine](https://pybullet.org/)

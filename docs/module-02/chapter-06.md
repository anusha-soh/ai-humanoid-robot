# Chapter 6: Building Your First Humanoid Simulation

## Overview

In this chapter, you'll bring your URDF robot models to life in Gazebo simulation. Building on the physics fundamentals from Chapter 5, you'll learn how to integrate your robot descriptions with Gazebo's simulation environment, add joint controllers to make your robot move, and create launch files that combine everything into a complete simulation. This is where your virtual robot truly comes to life!

You'll learn how to use Gazebo plugins to control your robot's joints, set up communication between ROS 2 and Gazebo, and create realistic humanoid behaviors. By the end of this chapter, you'll have a fully functional humanoid robot in simulation that responds to ROS 2 commands and moves realistically according to the physics parameters you've configured.

### Learning Objectives

By the end of this chapter, you will be able to:
- Import URDF robot models into Gazebo simulation
- Configure joint controllers for position, velocity, and effort control
- Use Gazebo plugins to enable ROS 2 communication with simulated robots
- Create launch files that start complete simulation environments
- Implement basic humanoid behaviors in simulation

### Prerequisites

Before starting this chapter, you should have:
- Completed Chapter 3 (URDF robot descriptions)
- Completed Chapter 5 (Physics simulation fundamentals)
- Understanding of ROS 2 topics and services
- Basic knowledge of robot control concepts

## Concepts

### Gazebo-ROS Integration

Gazebo integrates with ROS 2 through a set of plugins that allow communication between the simulation and the ROS 2 system. These plugins handle:

- **Robot State Publisher**: Publishes joint states from the simulation to ROS 2
- **Joint State Publisher**: Updates joint positions for visualization
- **Controller Manager**: Interfaces with ros2_control for hardware abstraction

### URDF to SDF Conversion

When a URDF model is loaded into Gazebo, it's converted to SDF (Simulation Description Format). During this conversion:

- URDF links become SDF links with physics properties
- URDF joints become SDF joints with dynamic properties
- Visual and collision properties are preserved
- Additional Gazebo-specific plugins can be added

### Joint Controllers

Gazebo uses various types of joint controllers:

- **Position Controllers**: Control joint position with PID feedback
- **Velocity Controllers**: Control joint velocity with PID feedback
- **Effort Controllers**: Apply direct force/torque to joints
- **Joint Trajectory Controllers**: Execute coordinated multi-joint trajectories

### Gazebo Plugins

Plugins extend Gazebo's functionality:

- **Model Plugins**: Affect entire models (e.g., controller interfaces)
- **Sensor Plugins**: Provide simulated sensor data
- **World Plugins**: Affect entire simulation world
- **GUI Plugins**: Extend the Gazebo interface

### ros2_control Framework

The ros2_control framework provides a standardized way to interface with both real and simulated hardware:

- **Hardware Interface**: Abstraction layer for hardware communication
- **Controller Manager**: Runtime controller loading and switching
- **Resource Manager**: Tracks available hardware resources
- **Transmission Interface**: Maps actuator positions to joint positions

## Examples

### Example 1: Adding Gazebo Plugins to URDF

First, let's enhance our URDF with Gazebo-specific plugins:

```xml
<?xml version="1.0" ?>
<robot name="humanoid_with_gazebo" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Include gazebo_ros plugin -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <parameters>$(find my_robot_description)/config/controllers.yaml</parameters>
    </plugin>
  </gazebo>

  <!-- Define materials -->
  <material name="blue">
    <color rgba="0.2 0.2 1.0 1.0"/>
  </material>
  <material name="red">
    <color rgba="1.0 0.2 0.2 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>

    <visual>
      <origin xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.2"/>
      </geometry>
      <material name="white"/>
    </visual>

    <collision>
      <origin xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.2"/>
      </geometry>
    </collision>
  </link>

  <!-- Torso -->
  <link name="torso">
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0.25"/>
      <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.5" iyz="0.0" izz="0.2"/>
    </inertial>

    <visual>
      <origin xyz="0 0 0.25"/>
      <geometry>
        <box size="0.2 0.2 0.5"/>
      </geometry>
      <material name="blue"/>
    </visual>

    <collision>
      <origin xyz="0 0 0.25"/>
      <geometry>
        <box size="0.2 0.2 0.5"/>
      </geometry>
    </collision>
  </link>

  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.1"/>
  </joint>

  <!-- Left arm -->
  <link name="left_upper_arm">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0.15"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>

    <visual>
      <origin xyz="0 0 0.15"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="red"/>
    </visual>

    <collision>
      <origin xyz="0 0 0.15"/>
      <geometry>
        <cylinder length="0.3" radius="0.3"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0 0.2"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="3.0"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>

  <!-- Add gazebo plugins for the joints -->
  <gazebo reference="left_shoulder_joint">
    <implicitSpringDamper>1</implicitSpringDamper>
  </gazebo>
</robot>
```

### Example 2: Controller Configuration File

Create a controller configuration file (config/controllers.yaml):

```yaml
# Controller manager configuration
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz
    use_sim_time: true

    # Available controllers
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    left_arm_controller:
      type: position_controllers/JointTrajectoryController

    right_arm_controller:
      type: position_controllers/JointTrajectoryController

# Left arm controller configuration
left_arm_controller:
  ros__parameters:
    joints:
      - left_shoulder_joint
      - left_elbow_joint
      - left_wrist_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity

# Right arm controller configuration
right_arm_controller:
  ros__parameters:
    joints:
      - right_shoulder_joint
      - right_elbow_joint
      - right_wrist_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

### Example 3: Launch File for Simulation

Create a launch file to start the complete simulation:

```python
#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    headless = LaunchConfiguration('headless', default='false')
    world = LaunchConfiguration('world', default='empty')

    # Package paths
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_robot_description = get_package_share_directory('my_robot_description')

    # World file path
    world_file = PathJoinSubstitution([
        FindPackageShare('my_robot_description'),
        'worlds',
        'simple_room.world'
    ])

    # Robot description (URDF)
    robot_desc_path = os.path.join(pkg_robot_description, 'urdf', 'humanoid_with_gazebo.urdf')
    with open(robot_desc_path, 'r') as infp:
        robot_desc = infp.read()

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_desc}
        ]
    )

    # Joint state publisher node
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_file,
            'gui': gui,
            'headless': headless,
            'verbose': 'false'
        }.items()
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'humanoid_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        output='screen'
    )

    # Controller manager
    controller_manager_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager-timeout', '60'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Left arm controller spawner
    left_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_arm_controller'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Right arm controller spawner
    right_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_arm_controller'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Use gui if true'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Use headless mode if true'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='empty',
            description='Choose one of the world files from `/my_robot_description/worlds`'
        ),
        robot_state_publisher,
        joint_state_publisher,
        gazebo,
        spawn_entity,
        controller_manager_spawner,
        left_arm_controller_spawner,
        right_arm_controller_spawner
    ])
```

### Example 4: Controlling the Robot in Simulation

Create a Python node to control the simulated robot:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import time


class HumanoidController(Node):
    """
    Node to control the simulated humanoid robot.
    Demonstrates sending trajectory commands to joint controllers.
    """

    def __init__(self):
        super().__init__('humanoid_controller')

        # Create publisher for left arm trajectory commands
        self.left_arm_pub = self.create_publisher(
            JointTrajectory,
            '/left_arm_controller/joint_trajectory',
            10
        )

        # Create publisher for right arm trajectory commands
        self.right_arm_pub = self.create_publisher(
            JointTrajectory,
            '/right_arm_controller/joint_trajectory',
            10
        )

        # Timer to send commands
        self.timer = self.create_timer(5.0, self.send_waving_motion)
        self.motion_cycle = 0

        self.get_logger().info('Humanoid controller node started')

    def send_waving_motion(self):
        # Define joint names for each arm
        left_joints = ['left_shoulder_joint', 'left_elbow_joint', 'left_wrist_joint']
        right_joints = ['right_shoulder_joint', 'right_elbow_joint', 'right_wrist_joint']

        # Create trajectory messages
        left_traj = JointTrajectory()
        left_traj.joint_names = left_joints
        left_traj.header.stamp = self.get_clock().now().to_msg()

        right_traj = JointTrajectory()
        right_traj.joint_names = right_joints
        right_traj.header.stamp = self.get_clock().now().to_msg()

        # Create trajectory point
        point = JointTrajectoryPoint()

        # Set positions based on motion cycle for waving motion
        if self.motion_cycle % 2 == 0:
            # Wave left arm
            point.positions = [0.5, -0.3, 0.2]  # Left arm waving position
            left_traj.points.append(point)

            # Keep right arm still
            point.positions = [0.0, 0.0, 0.0]   # Right arm neutral position
            right_traj.points.append(point)

            self.get_logger().info('Waving left arm')
        else:
            # Wave right arm
            point.positions = [0.0, 0.0, 0.0]   # Left arm neutral position
            left_traj.points.append(point)

            point.positions = [-0.5, 0.3, -0.2]  # Right arm waving position
            right_traj.points.append(point)

            self.get_logger().info('Waving right arm')

        # Set time from start (1 second for smooth motion)
        point.time_from_start = Duration(sec=1)

        # Publish trajectories
        self.left_arm_pub.publish(left_traj)
        self.right_arm_pub.publish(right_traj)

        self.motion_cycle += 1


def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Exercise

### Required Exercise: Complete Humanoid Simulation

Build a complete humanoid simulation with integrated control system:

**Task**: Create a simulation with:
1. A humanoid robot model with at least 6 degrees of freedom (3 per arm)
2. Proper controller configuration for all joints
3. A launch file that starts the complete simulation
4. A control node that makes the robot perform a coordinated waving behavior

**Acceptance Criteria**:
- Robot spawns correctly in Gazebo with proper physics properties
- All joints respond to ROS 2 trajectory commands
- Robot performs coordinated waving motion without joint violations
- Simulation runs smoothly at real-time speed

**Implementation Steps**:
1. Extend the URDF with additional joints for a complete humanoid
2. Create comprehensive controller configuration files
3. Build a launch file that initializes all required components
4. Implement a behavior node that coordinates the waving motion
5. Test the complete simulation system

**Extension Challenges**:
- **Beginner**: Add head movement to make the robot look around while waving
- **Intermediate**: Implement a walking gait simulation (if legs are added)
- **Advanced**: Create a learning system that improves the waving motion based on feedback

## References

- [Gazebo ROS Control](http://gazebosim.org/tutorials/?tut=ros2_control)
- [ros2_control Documentation](https://control.ros.org/)
- [Gazebo Plugins Guide](http://gazebosim.org/tutorials?tut=ros_comm)
- [URDF Gazebo Integration](http://gazebosim.org/tutorials?tut=ros_urdf)
- [Joint Trajectory Controllers](https://github.com/ros-controls/ros2_controllers)
- [Simulation Best Practices](http://gazebosim.org/tutorials?tut=sim_b3_exercises)

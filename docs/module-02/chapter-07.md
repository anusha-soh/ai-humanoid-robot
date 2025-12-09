# Chapter 7: Sensor Simulation - Eyes and Ears for Robots

## Overview

In this chapter, you'll learn how to equip your simulated robots with realistic sensors that provide the data needed for perception and navigation. Just as humans use their senses to understand the world, robots rely on sensors to perceive their environment. You'll learn how to simulate cameras, LiDAR, IMUs, and other sensors in Gazebo, configure their parameters to match real-world specifications, and process the resulting sensor data in ROS 2.

Sensor simulation is crucial for robotics development because it allows you to test perception algorithms without expensive hardware. You'll learn how to configure sensor noise models, set up realistic field-of-views, and create sensor fusion pipelines that combine multiple sensor modalities. By the end of this chapter, you'll have created a multi-sensor humanoid robot that can perceive its environment just like a real robot would.

### Learning Objectives

By the end of this chapter, you will be able to:
- Add simulated cameras, LiDAR, and IMU sensors to your robot models
- Configure sensor parameters to match real-world specifications
- Process sensor data streams in ROS 2 nodes
- Visualize sensor data in RViz2
- Implement basic sensor fusion techniques

### Prerequisites

Before starting this chapter, you should have:
- Completed Chapter 3 (URDF robot descriptions)
- Completed Chapter 6 (Gazebo simulation integration)
- Understanding of ROS 2 topics and message types
- Basic knowledge of computer vision and sensor principles

## Concepts

### Sensor Simulation in Gazebo

Gazebo provides realistic sensor simulation by modeling the physical properties of real sensors:

- **Camera Sensors**: Simulate RGB cameras with configurable resolution, field of view, and noise
- **Depth Sensors**: Provide depth information for 3D reconstruction and obstacle detection
- **LiDAR Sensors**: Simulate laser range finders for 2D/3D mapping and navigation
- **IMU Sensors**: Model inertial measurement units for orientation and acceleration

### Sensor Noise Models

Real sensors have imperfections that must be modeled in simulation:

- **Gaussian Noise**: Random variations in sensor readings
- **Bias**: Systematic offsets in sensor measurements
- **Drift**: Slow changes in sensor calibration over time
- **Outliers**: Occasional erroneous readings

### Sensor Data Processing

Sensor data in ROS 2 follows standardized message types:

- **sensor_msgs/Image**: Camera image data
- **sensor_msgs/LaserScan**: LiDAR scan data
- **sensor_msgs/Imu**: IMU orientation and acceleration data
- **sensor_msgs/PointCloud2**: 3D point cloud data

### Sensor Fusion

Combining data from multiple sensors improves perception accuracy:

- **Data Association**: Matching features across sensor modalities
- **Kalman Filtering**: Optimal combination of sensor measurements
- **Multi-modal Perception**: Leveraging complementary sensor capabilities

### Sensor Placement and Configuration

The placement and configuration of sensors affects robot perception:

- **Field of View**: Angular coverage of the sensor
- **Resolution**: Spatial detail captured by the sensor
- **Range**: Minimum and maximum distances the sensor can measure
- **Update Rate**: Frequency at which the sensor publishes data

## Examples

### Example 1: Adding Camera Sensors to URDF

Let's add a camera sensor to our humanoid robot:

```xml
<?xml version="1.0" ?>
<robot name="humanoid_with_sensors" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Gazebo plugin for ros control -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <parameters>$(find my_robot_description)/config/controllers.yaml</parameters>
    </plugin>
  </gazebo>

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
    </visual>

    <collision>
      <origin xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.2"/>
      </geometry>
    </collision>
  </link>

  <!-- Head link with camera -->
  <link name="head">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0.1"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.04"/>
    </inertial>

    <visual>
      <origin xyz="0 0 0.1"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>

    <collision>
      <origin xyz="0 0 0.1"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.2"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <!-- Camera link (fixed to head) -->
  <link name="camera_link">
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>

    <visual>
      <origin xyz="0 0 0"/>
      <geometry>
        <box size="0.02 0.04 0.02"/>
      </geometry>
    </visual>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="head"/>
    <child link="camera_link"/>
    <origin xyz="0.05 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo camera sensor plugin -->
  <gazebo reference="camera_link">
    <sensor name="camera" type="camera">
      <always_on>true</always_on>
      <update_rate>30</update_rate>
      <camera name="head_camera">
        <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10.0</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <frame_name>camera_link</frame_name>
        <min_depth>0.1</min_depth>
        <max_depth>10.0</max_depth>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

### Example 2: Adding LiDAR Sensor

Now let's add a LiDAR sensor to our robot:

```xml
  <!-- LiDAR link (on top of head) -->
  <link name="lidar_link">
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>

    <visual>
      <origin xyz="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.02"/>
      </geometry>
    </visual>
  </link>

  <joint name="lidar_joint" type="fixed">
    <parent link="head"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 0.25"/>
  </joint>

  <!-- Gazebo LiDAR sensor plugin -->
  <gazebo reference="lidar_link">
    <sensor name="lidar" type="ray">
      <always_on>true</always_on>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1.0</resolution>
            <min_angle>-3.14159</min_angle>  <!-- -π -->
            <max_angle>3.14159</max_angle>   <!-- π -->
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>10.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="lidar_controller" filename="libgazebo_ros_laser.so">
        <topic_name>/scan</topic_name>
        <frame_name>lidar_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
```

### Example 3: Adding IMU Sensor

Let's add an IMU sensor to our robot:

```xml
  <!-- IMU link (in the torso) -->
  <link name="imu_link">
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.00001" ixy="0.0" ixz="0.0" iyy="0.00001" iyz="0.0" izz="0.00001"/>
    </inertial>
  </link>

  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.05"/>
  </joint>

  <!-- Gazebo IMU sensor plugin -->
  <gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.01</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.01</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.01</stddev>
            </noise>
          </z>
        </angular_velocity>
        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.017</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.017</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.017</stddev>
            </noise>
          </z>
        </linear_acceleration>
      </imu>
      <plugin name="imu_controller" filename="libgazebo_ros_imu.so">
        <topic_name>/imu/data</topic_name>
        <frame_name>imu_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
```

### Example 4: Processing Sensor Data in Python

Create a Python node to process sensor data from the simulated robot:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist
import math


class SensorProcessor(Node):
    """
    Node to process data from multiple simulated sensors.
    Demonstrates sensor data fusion and processing techniques.
    """

    def __init__(self):
        super().__init__('sensor_processor')

        # Create subscribers for different sensor types
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )

        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Publisher for robot commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialize CvBridge for image processing
        self.bridge = CvBridge()

        # Store latest sensor data
        self.latest_image = None
        self.latest_scan = None
        self.latest_imu = None

        # Timer for processing loop
        self.timer = self.create_timer(0.1, self.process_sensors)

        self.get_logger().info('Sensor processor node started')

    def camera_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image

            # Example: Detect edges in the image
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)

            # Log image information
            height, width = cv_image.shape[:2]
            self.get_logger().info(f'Camera: {width}x{height} image received')
        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {e}')

    def lidar_callback(self, msg):
        """Process incoming LiDAR scan data"""
        self.latest_scan = msg

        # Analyze scan for obstacles
        if len(msg.ranges) > 0:
            # Get front-facing range (middle of scan)
            front_idx = len(msg.ranges) // 2
            front_distance = msg.ranges[front_idx]

            # Get minimum distance in front (±30 degrees)
            start_idx = len(msg.ranges) // 2 - 30
            end_idx = len(msg.ranges) // 2 + 30
            front_ranges = msg.ranges[start_idx:end_idx]
            front_ranges = [r for r in front_ranges if not math.isnan(r) and r != float('inf')]

            if front_ranges:
                min_front_dist = min(front_ranges)
                self.get_logger().info(f'LiDAR: Min front distance: {min_front_dist:.2f}m')

    def imu_callback(self, msg):
        """Process incoming IMU data"""
        self.latest_imu = msg

        # Extract orientation from quaternion
        orientation = msg.orientation
        # Convert quaternion to euler angles (simplified)
        # In practice, you'd use tf2 for this conversion
        self.get_logger().info(f'IMU: Orientation - x:{orientation.x:.3f}, y:{orientation.y:.3f}, z:{orientation.z:.3f}, w:{orientation.w:.3f}')

    def process_sensors(self):
        """Main processing loop that combines sensor data"""
        if self.latest_scan is not None:
            # Simple obstacle avoidance based on LiDAR
            msg = LaserScan()
            msg = self.latest_scan

            # Get front-facing range
            front_idx = len(msg.ranges) // 2
            front_distance = msg.ranges[front_idx]

            # Create velocity command based on sensor data
            cmd_msg = Twist()

            if front_distance < 1.0:  # Obstacle too close
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = 0.5  # Turn
                self.get_logger().warn('Obstacle detected - turning')
            else:
                cmd_msg.linear.x = 0.3  # Move forward
                cmd_msg.angular.z = 0.0
                self.get_logger().info('Clear path - moving forward')

            # Publish command
            self.cmd_pub.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorProcessor()

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

### Required Exercise: Multi-Sensor Perception System

Build a complete multi-sensor perception system for your humanoid robot:

**Task**: Create a system with:
1. A humanoid robot with camera, LiDAR, and IMU sensors
2. A sensor processing node that combines data from all sensors
3. A visualization setup that shows all sensor data in RViz2
4. A behavior that uses sensor fusion for navigation

**Acceptance Criteria**:
- All sensors publish realistic data streams
- Robot successfully navigates environment using sensor fusion
- Sensor data is properly visualized in RViz2
- System demonstrates complementary sensor capabilities

**Implementation Steps**:
1. Enhance your URDF with camera, LiDAR, and IMU sensors
2. Create a sensor processing node that fuses data from all sensors
3. Set up RViz2 configuration to visualize all sensor streams
4. Implement a navigation behavior that uses multiple sensor inputs
5. Test the complete system in simulation

**Extension Challenges**:
- **Beginner**: Add visualization markers to highlight detected objects
- **Intermediate**: Implement a simple SLAM system using LiDAR and IMU data
- **Advanced**: Create a machine learning model that processes sensor data for object recognition

## References

- [Gazebo Sensor Plugins](http://gazebosim.org/tutorials?tut=ros_gzplugins)
- [Camera Sensor Simulation](http://gazebosim.org/tutorials?tut=ros2_liveliness_camera)
- [LiDAR Simulation in Gazebo](http://gazebosim.org/tutorials?tut=ros_gzplugins_laser)
- [IMU Sensor Simulation](http://gazebosim.org/tutorials?tut=ros_gzplugins_imu)
- [Sensor Message Types](https://docs.ros.org/en/humble/p/sensor_msgs/)
- [Computer Vision with ROS](https://github.com/ros-perception/vision_opencv)

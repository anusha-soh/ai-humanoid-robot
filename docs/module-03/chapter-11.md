# Chapter 11: Visual SLAM and Localization (Isaac ROS)

## Overview

Welcome to Visual SLAM (Simultaneous Localization and Mapping), a critical capability for autonomous robots that need to navigate unknown environments. SLAM allows robots to simultaneously build maps of their surroundings while determining their position within those maps. In this chapter, you'll learn about Visual SLAM, which uses camera imagery to construct maps and estimate robot pose, and how to leverage NVIDIA's Isaac ROS for hardware-accelerated visual SLAM.

Visual SLAM differs from LiDAR-based SLAM in that it uses visual features extracted from camera images rather than laser range measurements. While LiDAR SLAM is more robust in low-texture environments, Visual SLAM offers advantages in terms of cost, power consumption, and the rich semantic information that can be extracted from visual data. You'll learn how to set up and configure Isaac ROS VSLAM pipelines, process visual odometry data, and integrate SLAM with navigation systems. By the end of this chapter, you'll have built an autonomous exploration system that can map environments and localize itself within them.

### Learning Objectives

By the end of this chapter, you will be able to:
- Explain the fundamentals of SLAM and its importance in robotics
- Implement Visual SLAM using Isaac ROS hardware-accelerated pipelines
- Configure and optimize VSLAM parameters for different environments
- Integrate SLAM with navigation and path planning systems
- Build autonomous exploration behaviors that map unknown environments

### Prerequisites

Before starting this chapter, you should have:
- Completed Modules 1-2 (ROS 2 basics, simulation, and perception)
- Understanding of computer vision fundamentals (features, matching)
- Basic knowledge of 3D geometry and pose estimation
- Experience with ROS 2 navigation systems

## Concepts

### SLAM Fundamentals

SLAM (Simultaneous Localization and Mapping) solves two problems simultaneously:
- **Mapping**: Building a map of an unknown environment
- **Localization**: Estimating the robot's position within that map

The challenge lies in the circular dependency: accurate localization requires a good map, while building a good map requires accurate localization.

### Visual SLAM vs LiDAR SLAM

Different SLAM approaches have distinct advantages:

- **Visual SLAM**:
  - Uses camera imagery to extract visual features
  - Lower cost and power consumption than LiDAR
  - Rich semantic information from visual data
  - Sensitive to lighting conditions and texture
  - Drift over long distances without loop closure

- **LiDAR SLAM**:
  - Uses laser range measurements
  - Robust to lighting variations
  - Accurate metric maps
  - Higher cost and power consumption
  - Less semantic information

### VSLAM Pipeline Components

A typical Visual SLAM pipeline includes:

- **Feature Detection**: Identifying distinctive points in images
- **Feature Tracking**: Following features across consecutive frames
- **Motion Estimation**: Calculating camera/robot motion
- **Bundle Adjustment**: Optimizing camera poses and 3D points
- **Loop Closure**: Detecting revisited locations to correct drift
- **Map Representation**: Maintaining a consistent map of the environment

### Isaac ROS VSLAM

Isaac ROS provides hardware-accelerated VSLAM capabilities:

- **Hardware Acceleration**: Utilizes NVIDIA GPUs for real-time performance
- **Modular Architecture**: Flexible pipeline components
- **ROS 2 Integration**: Seamless integration with ROS 2 ecosystem
- **Calibration Tools**: Proper camera calibration for accurate results

### Map Representation and Loop Closure

Critical components of SLAM systems:

- **Occupancy Grids**: 2D probabilistic maps of occupied/free space
- **Topological Maps**: Graph-based representations of key locations
- **Feature Maps**: Collections of visual features and their 3D positions
- **Loop Closure Detection**: Identifying when robot returns to known locations

### Pose Estimation and Optimization

Techniques for accurate pose estimation:

- **Visual Odometry**: Estimating motion between consecutive frames
- **Global Optimization**: Correcting accumulated errors through bundle adjustment
- **Sensor Fusion**: Combining visual data with IMU, wheel encoders, etc.
- **Robust Estimation**: Handling outliers and incorrect matches

## Examples

### Example 1: Setting up Isaac ROS VSLAM Pipeline

First, let's create a ROS 2 node that interfaces with Isaac ROS VSLAM:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformBroadcaster
import numpy as np
import cv2
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R


class VSLAMNode(Node):
    """
    Node that interfaces with Isaac ROS VSLAM pipeline.
    """

    def __init__(self):
        super().__init__('vslam_node')

        # Create subscriptions for camera images and camera info
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        # Create publishers for pose estimates and maps
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/vslam/pose',
            10
        )

        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/vslam/map',
            10
        )

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Store camera parameters
        self.camera_matrix = None
        self.distortion_coeffs = None

        # Store pose estimates
        self.current_pose = np.eye(4)  # 4x4 transformation matrix
        self.previous_features = None

        # Initialize feature detector
        self.feature_detector = cv2.SIFT_create()

        # TF broadcaster for robot pose
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('VSLAM node initialized')

    def camera_info_callback(self, msg):
        """Store camera calibration parameters"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.distortion_coeffs = np.array(msg.d)

    def image_callback(self, msg):
        """Process incoming camera image for VSLAM"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Extract features from current image
            keypoints, descriptors = self.feature_detector.detectAndCompute(cv_image, None)

            if self.previous_features is not None and descriptors is not None:
                # Match features with previous frame
                matches = self.match_features(
                    self.previous_features['descriptors'],
                    descriptors
                )

                if len(matches) >= 10:  # Need sufficient matches for pose estimation
                    # Estimate motion between frames
                    motion = self.estimate_motion(
                        self.previous_features['keypoints'],
                        keypoints,
                        matches,
                        self.camera_matrix
                    )

                    if motion is not None:
                        # Update current pose
                        self.current_pose = self.current_pose @ motion

                        # Publish pose estimate
                        self.publish_pose_estimate(msg.header, self.current_pose)

                        # Broadcast TF
                        self.broadcast_transform(msg.header, self.current_pose)

            # Store current features for next iteration
            self.previous_features = {
                'keypoints': keypoints,
                'descriptors': descriptors
            }

        except Exception as e:
            self.get_logger().error(f'Error processing VSLAM: {e}')

    def match_features(self, desc1, desc2):
        """Match features between two descriptor sets"""
        if desc1 is None or desc2 is None:
            return []

        # Use FLANN matcher for efficient feature matching
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)

        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(desc1, desc2, k=2)

        # Apply Lowe's ratio test to filter good matches
        good_matches = []
        for m, n in matches:
            if m.distance < 0.7 * n.distance:
                good_matches.append(m)

        return good_matches

    def estimate_motion(self, prev_keypoints, curr_keypoints, matches, camera_matrix):
        """Estimate motion between two frames using matched features"""
        if len(matches) < 10:
            return None

        # Get corresponding points
        prev_pts = np.float32([prev_keypoints[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        curr_pts = np.float32([curr_keypoints[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

        # Compute essential matrix
        E, mask = cv2.findEssentialMat(
            curr_pts, prev_pts, camera_matrix,
            method=cv2.RANSAC, prob=0.999, threshold=1.0
        )

        if E is None or E.size == 0:
            return None

        # Decompose essential matrix to get rotation and translation
        _, R, t, _ = cv2.recoverPose(E, curr_pts, prev_pts, camera_matrix)

        # Create transformation matrix
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t.flatten()

        return T

    def publish_pose_estimate(self, header, pose_matrix):
        """Publish pose estimate as PoseStamped message"""
        pose_msg = PoseStamped()
        pose_msg.header = header
        pose_msg.header.frame_id = 'map'

        # Extract position and orientation from transformation matrix
        pose_msg.pose.position.x = pose_matrix[0, 3]
        pose_msg.pose.position.y = pose_matrix[1, 3]
        pose_msg.pose.position.z = pose_matrix[2, 3]

        # Convert rotation matrix to quaternion
        r = R.from_matrix(pose_matrix[:3, :3])
        quat = r.as_quat()
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        self.pose_pub.publish(pose_msg)

    def broadcast_transform(self, header, pose_matrix):
        """Broadcast robot pose as TF transform"""
        t = TransformStamped()

        t.header.stamp = header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'vslam_odom'

        # Set translation
        t.transform.translation.x = pose_matrix[0, 3]
        t.transform.translation.y = pose_matrix[1, 3]
        t.transform.translation.z = pose_matrix[2, 3]

        # Set rotation
        r = R.from_matrix(pose_matrix[:3, :3])
        quat = r.as_quat()
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    vslam_node = VSLAMNode()

    try:
        rclpy.spin(vslam_node)
    except KeyboardInterrupt:
        pass
    finally:
        vslam_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Example 2: Creating an Occupancy Grid Map

Now let's create a node that builds an occupancy grid map from VSLAM data:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformListener, Buffer
import numpy as np
from scipy.ndimage import binary_dilation


class OccupancyGridMapper(Node):
    """
    Node that builds occupancy grid maps from VSLAM pose estimates and sensor data.
    """

    def __init__(self):
        super().__init__('occupancy_grid_mapper')

        # Create subscription to VSLAM pose estimates
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/vslam/pose',
            self.pose_callback,
            10
        )

        # Create subscription to laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Create publisher for the occupancy grid map
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            10
        )

        # Initialize TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Map parameters
        self.map_width = 400  # cells
        self.map_height = 400  # cells
        self.resolution = 0.05  # meters per cell
        self.origin_x = -10.0  # meters
        self.origin_y = -10.0  # meters

        # Initialize occupancy grid
        self.occupancy_grid = np.full((self.map_height, self.map_width), -1, dtype=np.int8)  # Unknown (-1)

        # Robot position
        self.robot_x = 0.0
        self.robot_y = 0.0

        # Timer for map publishing
        self.map_timer = self.create_timer(1.0, self.publish_map)

        self.get_logger().info('Occupancy Grid Mapper node initialized')

    def pose_callback(self, msg):
        """Update robot position from VSLAM pose estimate"""
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y

    def scan_callback(self, msg):
        """Process laser scan data to update occupancy grid"""
        try:
            # Transform scan to map frame if needed
            transform = self.tf_buffer.lookup_transform(
                'map',  # Target frame
                msg.header.frame_id,  # Source frame
                msg.header.stamp,  # Time
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().warn(f'Could not transform scan: {e}')
            return

        # Get robot position in map frame
        robot_map_x = int((self.robot_x - self.origin_x) / self.resolution)
        robot_map_y = int((self.robot_y - self.origin_y) / self.resolution)

        # Process laser scan beams
        for i, range_val in enumerate(msg.ranges):
            if not (msg.range_min <= range_val <= msg.range_max):
                continue  # Invalid range

            # Calculate beam angle
            angle = msg.angle_min + i * msg.angle_increment

            # Calculate endpoint of beam in world coordinates
            end_x = self.robot_x + range_val * np.cos(angle)
            end_y = self.robot_y + range_val * np.sin(angle)

            # Convert to map coordinates
            end_map_x = int((end_x - self.origin_x) / self.resolution)
            end_map_y = int((end_y - self.origin_y) / self.resolution)

            # Mark endpoint as occupied
            if 0 <= end_map_x < self.map_width and 0 <= end_map_y < self.map_height:
                self.occupancy_grid[end_map_y, end_map_x] = 100  # Occupied

            # Mark free space along the beam
            self.mark_free_space(robot_map_x, robot_map_y, end_map_x, end_map_y)

    def mark_free_space(self, start_x, start_y, end_x, end_y):
        """Mark free space along a beam using Bresenham's algorithm"""
        dx = abs(end_x - start_x)
        dy = abs(end_y - start_y)
        sx = 1 if start_x < end_x else -1
        sy = 1 if start_y < end_y else -1
        err = dx - dy

        x, y = start_x, start_y
        while True:
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                if self.occupancy_grid[y, x] == -1:  # If unknown, mark as free
                    self.occupancy_grid[y, x] = 0  # Free
                elif self.occupancy_grid[y, x] > 0:  # If occupied, reduce certainty slightly
                    self.occupancy_grid[y, x] = max(0, self.occupancy_grid[y, x] - 10)

            if x == end_x and y == end_y:
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def publish_map(self):
        """Publish the current occupancy grid map"""
        map_msg = OccupancyGrid()

        # Set header
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'

        # Set map metadata
        map_msg.info.resolution = self.resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        map_msg.info.origin.position.x = self.origin_x
        map_msg.info.origin.position.y = self.origin_y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        # Flatten grid for publishing (row-major order)
        map_msg.data = self.occupancy_grid.flatten().tolist()

        self.map_pub.publish(map_msg)
        self.get_logger().info(f'Published map: {self.map_width}x{self.map_height} grid')


def main(args=None):
    rclpy.init(args=args)
    mapper = OccupancyGridMapper()

    try:
        rclpy.spin(mapper)
    except KeyboardInterrupt:
        pass
    finally:
        mapper.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Example 3: Autonomous Exploration Behavior

Create a behavior node that performs autonomous exploration using VSLAM:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import numpy as np
import math
from collections import deque


class AutonomousExplorer(Node):
    """
    Node that implements autonomous exploration using VSLAM and mapping.
    """

    def __init__(self):
        super().__init__('autonomous_explorer')

        # Create subscription to robot pose
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/vslam/pose',
            self.pose_callback,
            10
        )

        # Create subscription to map
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Create publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create publisher for status updates
        self.status_pub = self.create_publisher(String, '/exploration_status', 10)

        # Timer for exploration control
        self.exploration_timer = self.create_timer(0.1, self.exploration_callback)

        # State variables
        self.current_pose = None
        self.current_map = None
        self.exploration_state = 'exploring'  # exploring, navigating_to_frontier, rotating
        self.frontiers = []  # List of potential exploration targets
        self.explored_cells = set()  # Track explored areas
        self.last_known_pose = None

        # Exploration parameters
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.min_frontier_distance = 1.0  # Minimum distance to consider a frontier
        self.exploration_timeout = 60.0  # Seconds before switching strategy
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        # Frontier detection parameters
        self.frontier_search_radius = 5.0  # meters to search for frontiers
        self.frontier_cell_count_threshold = 10  # minimum unknown cells for a frontier

        self.get_logger().info('Autonomous Explorer node initialized')

    def pose_callback(self, msg):
        """Update robot's current pose"""
        self.current_pose = msg.pose

    def map_callback(self, msg):
        """Update the current map"""
        # Reshape the flat map data back to 2D
        self.current_map = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.map_info = msg.info

    def exploration_callback(self):
        """Main exploration control loop"""
        if self.current_map is None or self.current_pose is None:
            # Wait for map and pose
            cmd_msg = Twist()
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_msg)
            return

        # Update exploration status
        exploration_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        if exploration_time > self.exploration_timeout:
            # Switch to rotation strategy if exploration is taking too long
            self.exploration_state = 'rotating'

        cmd_msg = Twist()

        if self.exploration_state == 'exploring':
            # Look for frontiers (boundaries between known and unknown space)
            frontier = self.find_next_frontier()

            if frontier is not None:
                # Navigate toward frontier
                cmd_msg = self.navigate_to_frontier(frontier)
                self.get_logger().info(f'Navigating to frontier at ({frontier[0]:.2f}, {frontier[1]:.2f})')
            else:
                # No frontiers found, rotate to explore surroundings
                self.exploration_state = 'rotating'
                cmd_msg.angular.z = self.angular_speed
                self.get_logger().info('No frontiers found, rotating to explore')

        elif self.exploration_state == 'rotating':
            # Rotate slowly to discover new areas
            cmd_msg.angular.z = self.angular_speed
            self.get_logger().info('Rotating to explore surroundings')

        # Publish velocity command
        self.cmd_vel_pub.publish(cmd_msg)

    def find_next_frontier(self):
        """Find the closest frontier point to explore"""
        if self.current_map is None:
            return None

        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y

        # Convert robot position to map coordinates
        robot_map_x = int((robot_x - self.map_info.origin.position.x) / self.map_info.resolution)
        robot_map_y = int((robot_y - self.map_info.origin.position.y) / self.map_info.resolution)

        # Search for frontiers in a radius around the robot
        search_radius = int(self.frontier_search_radius / self.map_info.resolution)
        search_radius = min(search_radius, min(self.current_map.shape) // 2)

        # Find frontier cells (adjacent to unknown space)
        frontiers = []
        for dx in range(-search_radius, search_radius + 1):
            for dy in range(-search_radius, search_radius + 1):
                x = robot_map_x + dx
                y = robot_map_y + dy

                if 0 <= x < self.current_map.shape[1] and 0 <= y < self.current_map.shape[0]:
                    if self.current_map[y, x] == 0:  # Free space
                        # Check if adjacent to unknown space
                        if self.has_unknown_neighbor(x, y):
                            world_x = x * self.map_info.resolution + self.map_info.origin.position.x
                            world_y = y * self.map_info.resolution + self.map_info.origin.position.y

                            distance = math.sqrt((world_x - robot_x)**2 + (world_y - robot_y)**2)
                            if distance >= self.min_frontier_distance:
                                frontiers.append((world_x, world_y, distance))

        # Return the closest frontier
        if frontiers:
            frontiers.sort(key=lambda f: f[2])  # Sort by distance
            return frontiers[0][:2]  # Return x, y coordinates

        return None

    def has_unknown_neighbor(self, x, y):
        """Check if a cell has unknown neighbors"""
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.current_map.shape[1] and 0 <= ny < self.current_map.shape[0]:
                    if self.current_map[ny, nx] == -1:  # Unknown
                        return True
        return False

    def navigate_to_frontier(self, frontier):
        """Generate velocity commands to navigate to a frontier point"""
        cmd_msg = Twist()

        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y

        # Calculate direction to frontier
        dx = frontier[0] - robot_x
        dy = frontier[1] - robot_y
        distance = math.sqrt(dx**2 + dy**2)

        if distance > 0.5:  # If not close to frontier
            # Move toward frontier
            cmd_msg.linear.x = min(self.linear_speed, distance * 0.5)  # Proportional to distance

            # Calculate angular velocity for heading correction
            target_angle = math.atan2(dy, dx)
            # In a real system, you'd get current yaw from pose orientation
            # For now, we'll use a simple proportional controller
            cmd_msg.angular.z = self.angular_speed * math.atan2(dy, dx) * 0.5

        return cmd_msg


def main(args=None):
    rclpy.init(args=args)
    explorer = AutonomousExplorer()

    try:
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot before shutting down
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.0
        cmd_msg.angular.z = 0.0
        explorer.cmd_vel_pub.publish(cmd_msg)

        explorer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Example 4: SLAM Integration Launch File

Create a launch file to bring up the complete SLAM system:

```python
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    camera_topic = LaunchConfiguration('camera_topic', default='/camera/image_raw')
    scan_topic = LaunchConfiguration('scan_topic', default='/scan')

    # Package names
    pkg_slam = get_package_share_directory('robot_slam')

    # Isaac ROS VSLAM node (simulated for this example)
    vslam_node = Node(
        package='robot_slam',
        executable='vslam_node',
        name='vslam_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'feature_type': 'sift'},  # Feature type to use
            {'max_features': 1000},    # Maximum features to track
            {'match_threshold': 0.7}   # Feature match threshold
        ],
        remappings=[
            ('/camera/image_raw', camera_topic),
        ],
        output='screen'
    )

    # Occupancy grid mapper node
    occupancy_mapper = Node(
        package='robot_slam',
        executable='occupancy_grid_mapper',
        name='occupancy_grid_mapper',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'map_resolution': 0.05},
            {'map_width': 400},
            {'map_height': 400}
        ],
        remappings=[
            ('/scan', scan_topic),
            ('/vslam/pose', '/vslam/pose')
        ],
        output='screen'
    )

    # Autonomous explorer node
    autonomous_explorer = Node(
        package='robot_slam',
        executable='autonomous_explorer',
        name='autonomous_explorer',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'linear_speed': 0.2},
            {'angular_speed': 0.5},
            {'exploration_timeout': 120.0}
        ],
        output='screen'
    )

    # AMCL localization node for comparison (when map is available)
    amcl_localization = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('robot_slam'),
                'config',
                'amcl_config.yaml'
            ]),
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/map', '/map'),  # Use the map from our VSLAM system
            ('/scan', scan_topic)
        ],
        output='screen'
    )

    # Map server to publish the map
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[
            {'yaml_filename': PathJoinSubstitution([
                FindPackageShare('robot_slam'),
                'maps',
                'explored_map.yaml'
            ])},
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # RViz2 for visualization
    rviz_config = PathJoinSubstitution([
        FindPackageShare('robot_slam'),
        'rviz',
        'slam_pipeline.rviz'
    ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/camera/image_raw',
            description='Topic name for camera feed'
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='Topic name for laser scan'
        ),
        vslam_node,
        occupancy_mapper,
        autonomous_explorer,
        # amcl_localization,  # Uncomment when map is available
        # map_server,  # Uncomment when map is available
        rviz
    ])
```

## Exercise

### Required Exercise: Complete VSLAM Mapping System

Build a complete Visual SLAM system that can autonomously explore and map environments:

**Task**: Create a system with:
1. A VSLAM pipeline that processes camera images and estimates robot pose
2. An occupancy grid mapper that builds maps from sensor data
3. An autonomous exploration behavior that discovers new areas
4. A launch file that integrates all components

**Acceptance Criteria**:
- VSLAM pipeline runs in real-time (10+ FPS) with stable pose estimates
- Occupancy grid map accurately represents environment structure
- Robot successfully explores unknown environments autonomously
- System handles loop closure and drift correction appropriately

**Implementation Steps**:
1. Implement the VSLAM node with feature detection and tracking
2. Create the occupancy grid mapper with sensor fusion
3. Build the autonomous exploration behavior with frontier detection
4. Create a launch file that starts the complete SLAM pipeline
5. Test the system in simulation with various environments

**Gazebo/Isaac Sim Fallback**: The system should work with both simulators:
- Use camera and LiDAR feeds from either Gazebo or Isaac Sim
- Ensure proper TF2 transforms for each simulator's coordinate system
- Validate that visual SLAM parameters work across both platforms

**Extension Challenges**:
- **Beginner**: Add visualization markers to show detected features and keyframes
- **Intermediate**: Implement loop closure detection to correct drift
- **Advanced**: Create a learning system that improves SLAM performance based on environmental features

## References

- [Visual SLAM Tutorial](https://arxiv.org/abs/1606.05830)
- [Isaac ROS VSLAM](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam)
- [ORB-SLAM Implementation](https://github.com/raulmur/ORB_SLAM2)
- [Occupancy Grid Mapping](https://ieeexplore.ieee.org/document/937792)
- [Frontier-Based Exploration](https://www.cs.cmu.edu/~motionplanning/reading/coverage/10.1.1.112.1835.pdf)
- [ROS 2 Navigation System](https://navigation.ros.org/)

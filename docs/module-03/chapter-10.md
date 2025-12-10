# Chapter 10: Perception Pipelines - Object Detection

## Overview

In this chapter, you'll learn how to build computer vision pipelines that enable your robots to perceive and understand objects in their environment. Object detection is a fundamental capability for autonomous robots, allowing them to identify, locate, and interact with objects in the real world. You'll learn how to implement YOLO (You Only Look Once) object detection in ROS 2, process camera feeds in real-time, and convert 2D detections to actionable 3D poses for robot manipulation.

Modern robotics perception relies heavily on deep learning models that can detect and classify objects in real-time. YOLO is one of the most popular real-time object detection algorithms due to its speed and accuracy. You'll learn how to integrate these models with your robot's perception system, handle image transport in ROS 2, and convert 2D bounding boxes to 3D world coordinates for manipulation tasks. By the end of this chapter, you'll have built a complete perception pipeline that can detect objects and guide your robot to fetch them.

### Learning Objectives

By the end of this chapter, you will be able to:
- Implement real-time object detection using YOLO in ROS 2
- Process camera feeds and handle image transport efficiently
- Convert 2D bounding boxes to 3D poses for robot manipulation
- Build complete perception-action loops for object fetching
- Optimize inference pipelines for real-time performance

### Prerequisites

Before starting this chapter, you should have:
- Completed Modules 1 and 2 (ROS 2 basics and simulation)
- Understanding of basic computer vision concepts
- Familiarity with Python and deep learning frameworks
- Basic knowledge of 3D geometry and coordinate transformations

## Concepts

### Object Detection Fundamentals

Object detection combines object classification and localization:

- **Classification**: Identifying what object is present
- **Localization**: Determining where the object is located
- **Bounding Boxes**: Rectangular regions that enclose detected objects
- **Confidence Scores**: Probability that the detection is correct

### YOLO Architecture

YOLO (You Only Look Once) processes the entire image in a single forward pass:

- **Single Network**: Processes full image at once rather than sliding window
- **Grid Division**: Divides image into grid cells for detection
- **Bounding Box Prediction**: Each cell predicts multiple bounding boxes
- **Class Probabilities**: Each box has class probability predictions
- **Real-time Performance**: Optimized for speed while maintaining accuracy

### ROS 2 Image Transport

ROS 2 provides several mechanisms for image transport:

- **sensor_msgs/Image**: Raw image data with metadata
- **compressed_image_transport**: Compressed formats (JPEG, PNG) for bandwidth efficiency
- **theora_image_transport**: Video compression for streaming
- **CameraInfo**: Calibration and projection matrices for 3D reconstruction

### 2D-to-3D Pose Estimation

Converting 2D detections to 3D world coordinates:

- **Camera Calibration**: Intrinsic parameters (focal length, principal point)
- **Projection Matrix**: Relationship between 3D world and 2D image
- **Depth Information**: Either from stereo, depth sensors, or geometric assumptions
- **Coordinate Transformation**: Converting from image frame to robot/world frame

### Perception-Action Integration

Connecting perception outputs to robot actions:

- **Detection Filtering**: Removing low-confidence or irrelevant detections
- **Object Selection**: Choosing which object to interact with
- **Pose Estimation**: Calculating grasp or approach poses
- **Behavior Execution**: Triggering appropriate robot behaviors

## Examples

### Example 1: Setting up YOLO Object Detection Node

First, let's create a ROS 2 node that runs YOLO object detection:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import Point


class YOLODetector(Node):
    """
    ROS 2 node that performs YOLO object detection on camera images.
    """

    def __init__(self):
        super().__init__('yolo_detector')

        # Create subscription to camera image
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Create subscription to camera info
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        # Create publisher for detections
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/detections',
            10
        )

        # Create publisher for annotated image
        self.annotated_image_pub = self.create_publisher(
            Image,
            '/camera/image_annotated',
            10
        )

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Load YOLO model
        self.model = YOLO('yolov8n.pt')  # You can use other YOLO variants

        # Store camera parameters
        self.camera_matrix = None
        self.distortion_coeffs = None

        # Detection confidence threshold
        self.confidence_threshold = 0.5

        self.get_logger().info('YOLO Detector node initialized')

    def camera_info_callback(self, msg):
        """Store camera calibration parameters"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.distortion_coeffs = np.array(msg.d)

    def image_callback(self, msg):
        """Process incoming camera image and perform object detection"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Run YOLO detection
            results = self.model(cv_image)

            # Create detection array message
            detection_array = Detection2DArray()
            detection_array.header = msg.header

            # Process detections
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        # Extract bounding box coordinates
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                        confidence = float(box.conf[0])
                        class_id = int(box.cls[0])

                        # Filter by confidence
                        if confidence > self.confidence_threshold:
                            # Create detection message
                            detection = Detection2D()
                            detection.header = msg.header

                            # Set bounding box
                            bbox = detection.bbox
                            bbox.center.x = (x1 + x2) / 2.0
                            bbox.center.y = (y1 + y2) / 2.0
                            bbox.size_x = x2 - x1
                            bbox.size_y = y2 - y1

                            # Set object hypothesis
                            hypothesis = ObjectHypothesisWithPose()
                            hypothesis.hypothesis.class_id = str(class_id)
                            hypothesis.hypothesis.score = confidence
                            detection.results.append(hypothesis)

                            detection_array.detections.append(detection)

            # Publish detections
            self.detection_pub.publish(detection_array)

            # Draw annotations on image
            annotated_image = self.annotate_image(cv_image, results)

            # Publish annotated image
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            annotated_msg.header = msg.header
            self.annotated_image_pub.publish(annotated_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def annotate_image(self, image, results):
        """Draw bounding boxes and labels on the image"""
        annotated_image = image.copy()

        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    confidence = float(box.conf[0])
                    class_id = int(box.cls[0])

                    # Get class name (simplified - you'd use a proper mapping)
                    class_name = self.model.names[class_id]

                    # Draw bounding box
                    cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    # Draw label
                    label = f'{class_name}: {confidence:.2f}'
                    cv2.putText(annotated_image, label, (x1, y1 - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return annotated_image


def main(args=None):
    rclpy.init(args=args)
    detector = YOLODetector()

    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Example 2: Converting 2D Detections to 3D Positions

Now let's create a node that converts 2D detections to 3D world coordinates:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_point
import numpy as np
from scipy.spatial.transform import Rotation as R


class Detection3DConverter(Node):
    """
    Node that converts 2D object detections to 3D world coordinates.
    """

    def __init__(self):
        super().__init__('detection_3d_converter')

        # Create subscription to 2D detections
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10
        )

        # Create publisher for 3D object positions
        self.object_pos_pub = self.create_publisher(
            PointStamped,
            '/detected_object_position',
            10
        )

        # Initialize TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Camera parameters (these should come from camera_info topic)
        self.fx = 554.25  # Focal length x
        self.fy = 554.25  # Focal length y
        self.cx = 320.0   # Principal point x
        self.cy = 240.0   # Principal point y

        # Default depth assumption (in meters)
        self.default_depth = 1.0

        self.get_logger().info('Detection 3D Converter node initialized')

    def detection_callback(self, msg):
        """Process 2D detections and convert to 3D positions"""
        for detection in msg.detections:
            # Get 2D position from detection
            x_2d = detection.bbox.center.x
            y_2d = detection.bbox.center.y

            # Convert 2D pixel coordinates to 3D world coordinates
            # This is a simplified approach - in practice, you'd use depth info
            point_3d = self.pixel_to_world(x_2d, y_2d, self.default_depth)

            # Create PointStamped message
            point_stamped = PointStamped()
            point_stamped.header = msg.header
            point_stamped.point.x = point_3d[0]
            point_stamped.point.y = point_3d[1]
            point_stamped.point.z = point_3d[2]

            # Transform to robot base frame if needed
            try:
                # Wait for transform
                transform = self.tf_buffer.lookup_transform(
                    'base_link',  # Target frame
                    msg.header.frame_id,  # Source frame
                    rclpy.time.Time(),  # Time (0 = latest)
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )

                # Apply transform
                transformed_point = do_transform_point(point_stamped, transform)
                point_stamped = transformed_point

            except Exception as e:
                self.get_logger().warn(f'Transform failed: {e}')

            # Publish 3D position
            self.object_pos_pub.publish(point_stamped)

            # Log detection
            class_id = detection.results[0].hypothesis.class_id if detection.results else "unknown"
            confidence = detection.results[0].hypothesis.score if detection.results else 0.0
            self.get_logger().info(
                f'Detected {class_id} at 3D position: '
                f'({point_stamped.point.x:.2f}, {point_stamped.point.y:.2f}, {point_stamped.point.z:.2f}), '
                f'confidence: {confidence:.2f}'
            )

    def pixel_to_world(self, x_pixel, y_pixel, depth):
        """
        Convert 2D pixel coordinates to 3D world coordinates using camera parameters.
        This is a simplified pinhole camera model.
        """
        # Convert pixel coordinates to camera frame
        x_cam = (x_pixel - self.cx) * depth / self.fx
        y_cam = (y_pixel - self.cy) * depth / self.fy
        z_cam = depth

        return np.array([x_cam, y_cam, z_cam])


def main(args=None):
    rclpy.init(args=args)
    converter = Detection3DConverter()

    try:
        rclpy.spin(converter)
    except KeyboardInterrupt:
        pass
    finally:
        converter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Example 3: Object Fetching Behavior

Create a behavior node that uses object detection to fetch objects:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist, Pose
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String
from builtin_interfaces.msg import Duration
import numpy as np
import math


class ObjectFetcher(Node):
    """
    Node that implements object fetching behavior using perception.
    """

    def __init__(self):
        super().__init__('object_fetcher')

        # Create subscription to 3D object positions
        self.object_pos_sub = self.create_subscription(
            PointStamped,
            '/detected_object_position',
            self.object_position_callback,
            10
        )

        # Create subscription to detections for filtering
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10
        )

        # Create publisher for robot velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create publisher for status updates
        self.status_pub = self.create_publisher(String, '/fetch_status', 10)

        # Timer for navigation control
        self.nav_timer = self.create_timer(0.1, self.navigation_callback)

        # State variables
        self.target_object = None
        self.target_position = None
        self.navigation_state = 'searching'  # searching, navigating, approaching, grasping
        self.target_class = 'cup'  # We're looking for cups
        self.min_confidence = 0.7

        # Navigation parameters
        self.linear_speed = 0.3
        self.angular_speed = 0.5
        self.approach_distance = 0.5  # Distance to stop before object
        self.reached_threshold = 0.1  # Threshold to consider reached

        self.get_logger().info('Object Fetcher node initialized')

    def detection_callback(self, msg):
        """Process detections to identify target object"""
        for detection in msg.detections:
            # Check if this is our target class and confidence is high enough
            if detection.results and len(detection.results) > 0:
                class_id = detection.results[0].hypothesis.class_id
                confidence = detection.results[0].hypothesis.score

                if class_id == self.target_class and confidence >= self.min_confidence:
                    # This is our target object
                    self.get_logger().info(f'Found target {self.target_class} with confidence {confidence:.2f}')
                    self.status_pub.publish(String(data=f'Found {self.target_class}'))
                    return  # Found our target, no need to check others

    def object_position_callback(self, msg):
        """Process object position and update target"""
        # Check if this is the object we're looking for
        # In a real system, you'd have a more sophisticated matching system
        self.target_position = msg.point
        self.navigation_state = 'navigating'
        self.get_logger().info(
            f'Navigating to {self.target_class} at: '
            f'({msg.point.x:.2f}, {msg.point.y:.2f}, {msg.point.z:.2f})'
        )

    def navigation_callback(self):
        """Main navigation control loop"""
        if self.target_position is None:
            # No target detected, continue searching
            self.search_behavior()
            return

        # Calculate distance to target
        distance_to_target = math.sqrt(
            self.target_position.x**2 +
            self.target_position.y**2 +
            self.target_position.z**2
        )

        cmd_msg = Twist()

        if self.navigation_state == 'navigating':
            if distance_to_target > self.approach_distance:
                # Navigate toward object
                cmd_msg.linear.x = self.linear_speed
                # Simple proportional controller for angular alignment
                angle_to_target = math.atan2(self.target_position.y, self.target_position.x)
                cmd_msg.angular.z = self.angular_speed * angle_to_target
                self.get_logger().info(f'Navigating: distance {distance_to_target:.2f}m')
            else:
                # Close enough, approach carefully
                self.navigation_state = 'approaching'
                self.get_logger().info('Approaching target object')

        elif self.navigation_state == 'approaching':
            if distance_to_target > self.reached_threshold:
                # Approach slowly
                cmd_msg.linear.x = 0.1  # Slow approach
                angle_to_target = math.atan2(self.target_position.y, self.target_position.x)
                cmd_msg.angular.z = self.angular_speed * angle_to_target * 0.5
                self.get_logger().info(f'Approaching: distance {distance_to_target:.2f}m')
            else:
                # Reached target
                self.navigation_state = 'reached'
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = 0.0
                self.get_logger().info('Target reached! Ready to grasp.')
                self.status_pub.publish(String(data='Target reached - ready to grasp'))

        # Publish velocity command
        self.cmd_vel_pub.publish(cmd_msg)

    def search_behavior(self):
        """Behavior when no target is detected"""
        if self.navigation_state == 'searching':
            # Rotate slowly to search for objects
            cmd_msg = Twist()
            cmd_msg.angular.z = 0.2  # Slow rotation
            self.cmd_vel_pub.publish(cmd_msg)
            self.get_logger().info('Searching for target object...')


def main(args=None):
    rclpy.init(args=args)
    fetcher = ObjectFetcher()

    try:
        rclpy.spin(fetcher)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot before shutting down
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.0
        cmd_msg.angular.z = 0.0
        fetcher.cmd_vel_pub.publish(cmd_msg)

        fetcher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Example 4: Complete Perception Pipeline Launch

Create a launch file to bring up the complete perception pipeline:

```python
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    camera_topic = LaunchConfiguration('camera_topic', default='/camera/image_raw')
    detection_topic = LaunchConfiguration('detection_topic', default='/detections')

    # Package names
    pkg_perception = get_package_share_directory('robot_perception')

    # YOLO detector node
    yolo_detector = Node(
        package='robot_perception',
        executable='yolo_detector',
        name='yolo_detector',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'confidence_threshold': 0.5}
        ],
        remappings=[
            ('/camera/image_raw', camera_topic),
            ('/detections', detection_topic)
        ],
        output='screen'
    )

    # 3D converter node
    detection_3d_converter = Node(
        package='robot_perception',
        executable='detection_3d_converter',
        name='detection_3d_converter',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/detections', detection_topic)
        ],
        output='screen'
    )

    # Object fetcher node
    object_fetcher = Node(
        package='robot_perception',
        executable='object_fetcher',
        name='object_fetcher',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'target_class': 'cup'},
            {'min_confidence': 0.7}
        ],
        output='screen'
    )

    # RViz2 node for visualization
    rviz_config = PathJoinSubstitution([
        FindPackageShare('robot_perception'),
        'rviz',
        'perception_pipeline.rviz'
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
            'detection_topic',
            default_value='/detections',
            description='Topic name for detection results'
        ),
        yolo_detector,
        detection_3d_converter,
        object_fetcher,
        rviz
    ])
```

## Exercise

### Required Exercise: Complete Object Fetching System

Build a complete perception-action system that can detect and fetch objects:

**Task**: Create a system with:
1. A YOLO-based object detection pipeline that runs in real-time
2. 2D-to-3D conversion for detected objects
3. A navigation system that moves the robot toward detected objects
4. A complete launch file that starts all required nodes

**Acceptance Criteria**:
- Object detection runs at 10+ FPS on standard hardware
- 2D detections are accurately converted to 3D world coordinates
- Robot successfully navigates to detected objects
- System handles multiple object classes and prioritizes targets

**Implementation Steps**:
1. Implement the YOLO detection node with proper ROS 2 interfaces
2. Create the 3D conversion node with TF2 integration
3. Build the navigation behavior with obstacle avoidance
4. Create a launch file that starts the complete pipeline
5. Test the system in simulation with various objects

**Gazebo/Isaac Sim Fallback**: The system should work with both simulators:
- Use simulated camera feeds from either Gazebo or Isaac Sim
- Ensure TF2 transforms are properly configured for each simulator
- Validate that depth information is appropriately handled

**Extension Challenges**:
- **Beginner**: Add more object classes to the detection system
- **Intermediate**: Implement grasp pose estimation for picked objects
- **Advanced**: Create a learning system that improves detection accuracy based on robot interactions

## References

- [YOLO Object Detection](https://pjreddie.com/darknet/yolo/)
- [Ultralytics YOLOv8](https://docs.ultralytics.com/)
- [ROS 2 Vision Messages](https://github.com/ros-perception/vision_msgs)
- [TF2 Tutorials](http://wiki.ros.org/tf2/Tutorials)
- [OpenCV with ROS](https://wiki.ros.org/vision_opencv)
- [Real-time Object Detection](https://arxiv.org/abs/1506.02640)

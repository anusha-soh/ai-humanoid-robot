# Chapter 4: Bridging Python AI to ROS 2 (rclpy)

## Overview

In the previous chapters, you learned about ROS 2 communication patterns and robot description formats. Now it's time to bridge the gap between your Python AI code and the ROS 2 ecosystem. The rclpy library provides Python bindings for ROS 2, allowing you to create nodes, publish and subscribe to topics, and call services directly from Python.

This chapter is crucial because most AI practitioners work in Python, and rclpy enables seamless integration between AI algorithms and robotic systems. You'll learn how to create Python nodes that can send commands to robots and receive sensor data, implement reactive behaviors that respond to environmental changes, and build AI agents that can control robots autonomously. By the end of this chapter, you'll have created a Python-based AI agent that successfully avoids obstacles in simulation.

### Learning Objectives

By the end of this chapter, you will be able to:
- Create ROS 2 nodes using the rclpy library
- Publish robot commands from Python code
- Subscribe to sensor data in Python nodes
- Implement reactive behaviors that respond to sensor input
- Integrate Python AI algorithms with ROS 2 systems

### Prerequisites

Before starting this chapter, you should have:
- Completed Chapters 1-3 (ROS 2 basics, communication, and URDF)
- Proficiency in Python programming (classes, functions, modules)
- Understanding of basic AI/ML concepts
- Familiarity with NumPy for numerical computations
- Basic understanding of robot control concepts

## Concepts

### What is rclpy?

**rclpy** is the Python client library for ROS 2. It provides Python bindings for the ROS 2 client library (rcl), allowing Python developers to interact with ROS 2 systems. rclpy enables:

- Creating ROS 2 nodes in Python
- Publishing and subscribing to topics
- Creating and calling services
- Creating action clients and servers
- Managing parameters and logging

### Node Structure in rclpy

A typical rclpy node follows this structure:
1. Initialize the ROS 2 client library
2. Create a node instance
3. Create publishers, subscribers, services, etc.
4. Spin the node to process callbacks
5. Clean up and shutdown

### Publishers and Subscribers in Python

In rclpy, publishers and subscribers are created using the node's `create_publisher()` and `create_subscription()` methods. Publishers send messages to topics, while subscribers receive messages from topics through callback functions.

### Services and Clients in Python

Services allow for request-response communication. A service server is created using `create_service()`, and a client is created using `create_client()`. Clients call services using asynchronous or synchronous methods.

### Parameters in rclpy

Parameters allow nodes to be configured at runtime. They can be set via launch files, command line, or other nodes, and accessed using the node's parameter system.

### Threading and Callbacks

rclpy handles threading internally, but it's important to understand how callbacks are executed and how to avoid blocking the main thread. The executor manages callback execution and can be single-threaded or multi-threaded.

## Examples

### Example 1: Creating a Basic Publisher Node in Python

Let's create a Python node that publishes joint commands to move a robot:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np


class JointCommandPublisher(Node):
    """
    Python node that publishes joint commands to control a robot.
    Demonstrates basic publisher functionality in rclpy.
    """

    def __init__(self):
        super().__init__('joint_command_publisher')

        # Create publisher for joint commands
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_commands',
            10
        )

        # Timer to send commands at regular intervals
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize joint positions
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]  # 4 joints
        self.time = 0.0

        self.get_logger().info('Joint command publisher node started')

    def timer_callback(self):
        # Generate oscillating joint commands
        self.time += 0.1
        self.joint_positions[0] = 0.5 * np.sin(self.time)  # Shoulder
        self.joint_positions[1] = 0.3 * np.cos(self.time)  # Elbow
        self.joint_positions[2] = 0.2 * np.sin(self.time * 2)  # Wrist
        self.joint_positions[3] = 0.1 * np.cos(self.time * 2)  # Gripper

        # Create and publish message
        msg = Float64MultiArray()
        msg.data = self.joint_positions
        self.publisher.publish(msg)

        self.get_logger().info(f'Published joint positions: {self.joint_positions}')


def main(args=None):
    rclpy.init(args=args)
    node = JointCommandPublisher()

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

### Example 2: Creating a Subscriber Node for Sensor Data

Now let's create a Python node that subscribes to sensor data:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np


class SensorSubscriber(Node):
    """
    Python node that subscribes to laser scan data.
    Demonstrates subscriber functionality in rclpy.
    """

    def __init__(self):
        super().__init__('sensor_subscriber')

        # Create subscription to laser scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/laser_scan',
            self.laser_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Store latest sensor data
        self.latest_scan = None

        self.get_logger().info('Sensor subscriber node started')

    def laser_callback(self, msg):
        # Store the latest scan data
        self.latest_scan = msg

        # Process the scan data
        ranges = np.array(msg.ranges)

        # Filter out invalid ranges (inf or nan)
        valid_ranges = ranges[np.isfinite(ranges)]

        if len(valid_ranges) > 0:
            min_distance = np.min(valid_ranges)
            avg_distance = np.mean(valid_ranges)

            self.get_logger().info(
                f'Min distance: {min_distance:.2f}m, '
                f'Avg distance: {avg_distance:.2f}m'
            )

            # Check for obstacles close to the robot
            if min_distance < 1.0:  # Less than 1 meter
                self.get_logger().warn(f'Obstacle detected at {min_distance:.2f}m!')


def main(args=None):
    rclpy.init(args=args)
    node = SensorSubscriber()

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

### Example 3: Creating a Reactive Behavior Node

Let's create a node that implements reactive obstacle avoidance behavior:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np


class ReactiveAvoidance(Node):
    """
    Python node that implements reactive obstacle avoidance.
    Combines subscription and publishing to control robot motion.
    """

    def __init__(self):
        super().__init__('reactive_avoidance')

        # Create subscription to laser scan
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/laser_scan',
            self.scan_callback,
            10
        )

        # Create publisher for velocity commands
        self.cmd_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Store latest scan and command
        self.latest_scan = None
        self.obstacle_detected = False

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # Robot parameters
        self.safe_distance = 1.0  # meters
        self.linear_speed = 0.5   # m/s
        self.angular_speed = 0.8  # rad/s

        self.get_logger().info('Reactive avoidance node started')

    def scan_callback(self, msg):
        self.latest_scan = msg
        ranges = np.array(msg.ranges)

        # Check for obstacles in front of the robot (front 60 degrees)
        front_ranges = ranges[len(ranges)//2 - 30 : len(ranges)//2 + 30]
        front_ranges = front_ranges[np.isfinite(front_ranges)]

        if len(front_ranges) > 0:
            min_front_distance = np.min(front_ranges)
            self.obstacle_detected = min_front_distance < self.safe_distance
        else:
            self.obstacle_detected = False

    def control_loop(self):
        if self.latest_scan is None:
            return

        cmd_msg = Twist()

        if self.obstacle_detected:
            # Turn to avoid obstacle
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = self.angular_speed
            self.get_logger().info('Obstacle detected - turning')
        else:
            # Move forward
            cmd_msg.linear.x = self.linear_speed
            cmd_msg.angular.z = 0.0
            self.get_logger().info('Clear path - moving forward')

        self.cmd_publisher.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveAvoidance()

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

### Example 4: Integrating AI with ROS 2

Now let's create a more sophisticated AI node that uses simple machine learning to process sensor data:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
from sklearn.cluster import DBSCAN


class AIBasedNavigation(Node):
    """
    Python node that uses simple AI to process sensor data and navigate.
    Demonstrates AI integration with ROS 2.
    """

    def __init__(self):
        super().__init__('ai_navigation')

        # Create subscription to laser scan
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/laser_scan',
            self.scan_callback,
            10
        )

        # Create publisher for velocity commands
        self.cmd_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Create publisher for AI status
        self.status_publisher = self.create_publisher(
            String,
            '/ai_status',
            10
        )

        # Store latest scan and state
        self.latest_scan = None
        self.navigation_state = 'exploring'  # exploring, avoiding, goal_reached

        # Timer for AI processing
        self.timer = self.create_timer(0.2, self.ai_processing_loop)

        # Robot parameters
        self.safe_distance = 0.8
        self.linear_speed = 0.4
        self.angular_speed = 0.6

        self.get_logger().info('AI-based navigation node started')

    def scan_callback(self, msg):
        self.latest_scan = msg

    def ai_processing_loop(self):
        if self.latest_scan is None:
            return

        # Process laser scan using AI techniques
        ranges = np.array(self.latest_scan.ranges)
        angles = np.linspace(
            self.latest_scan.angle_min,
            self.latest_scan.angle_max,
            len(ranges)
        )

        # Filter out invalid ranges
        valid_indices = np.isfinite(ranges)
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]

        # Convert to Cartesian coordinates
        x_points = valid_ranges * np.cos(valid_angles)
        y_points = valid_ranges * np.sin(valid_angles)

        # Cluster points to identify obstacles
        if len(x_points) > 0:
            points = np.column_stack((x_points, y_points))
            clustering = DBSCAN(eps=0.5, min_samples=3)
            cluster_labels = clustering.fit_predict(points)

            # Analyze clusters to determine navigation strategy
            obstacle_clusters = cluster_labels[cluster_labels != -1]
            if len(obstacle_clusters) > 0:
                self.navigation_state = 'avoiding'
            else:
                self.navigation_state = 'exploring'

        # Generate navigation commands based on AI analysis
        cmd_msg = Twist()
        status_msg = String()

        if self.navigation_state == 'avoiding':
            # Implement obstacle avoidance
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = self.angular_speed
            status_msg.data = 'AI: Avoiding obstacles'
        else:
            # Move forward
            cmd_msg.linear.x = self.linear_speed
            cmd_msg.angular.z = 0.0
            status_msg.data = 'AI: Exploring environment'

        self.cmd_publisher.publish(cmd_msg)
        self.status_publisher.publish(status_msg)

        self.get_logger().info(status_msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = AIBasedNavigation()

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

### Required Exercise: Python AI Agent for Obstacle Avoidance

Build a complete Python AI agent that integrates sensor processing, decision making, and robot control:

**Task**: Create an AI agent with:
1. A sensor processing module that analyzes laser scan data
2. A decision-making module that selects appropriate behaviors
3. A control module that sends commands to the robot
4. A state machine that manages different navigation states

**Acceptance Criteria**:
- AI agent successfully navigates environment avoiding obstacles
- Robot maintains safe distance from obstacles (> 0.8m)
- Agent transitions smoothly between exploration and avoidance states
- Code follows rclpy best practices and error handling

**Implementation Steps**:
1. Create a single Python node that combines all AI agent components
2. Implement sensor data processing with noise filtering
3. Create a finite state machine for navigation behavior
4. Test the agent in a simulated environment with obstacles

**Extension Challenges**:
- **Beginner**: Add visualization of AI decision-making process
- **Intermediate**: Implement path planning to navigate around obstacles systematically
- **Advanced**: Add learning capabilities that improve navigation over time

## References

- [rclpy Documentation](https://docs.ros.org/en/humble/p/rclpy/)
- [Python Client Library for ROS 2](https://github.com/ros2/rclpy)
- [ROS 2 Python Tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [Sensor Message Types](https://docs.ros.org/en/humble/p/sensor_msgs/)
- [Python AI Integration with ROS](https://navigation.ros.org/tutorials/docs/get_back_to_center.html)
- [Robot Control with Python](https://github.com/ros-controls/ros2_control)

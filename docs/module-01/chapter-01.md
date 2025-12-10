# Chapter 1: Welcome to Physical AI

## Overview

Welcome to the exciting world of Physical AI! This field represents a revolutionary convergence of artificial intelligence and robotics, where intelligent algorithms are embodied in physical systems that can interact with the real world. Unlike traditional "digital" AI that processes information on screens, Physical AI brings intelligence into three-dimensional space through robots that can sense, think, and act in their environment.

In this chapter, you'll embark on a journey to understand the fundamental concepts that make robots possible. You'll learn about the sense-think-act loop that powers all autonomous systems, explore why robots need specialized communication middleware, and install ROS 2 (Robot Operating System 2) - the industry-standard framework for building robotic applications. By the end of this chapter, you'll have created your first ROS 2 node and witnessed the magic of distributed robotics communication.

### Learning Objectives

By the end of this chapter, you will be able to:
- Define Physical AI and distinguish it from traditional digital AI
- Explain the sense-think-act loop that governs all robotic systems
- Install and verify ROS 2 Humble Hawksbill on your system
- Create and run a simple ROS 2 publisher node that communicates with other nodes
- Visualize the computation graph of ROS 2 nodes using rqt_graph

### Prerequisites

Before starting this chapter, you should have:
- Intermediate Python programming skills (classes, functions, modules)
- Basic understanding of network concepts (publishing, subscribing)
- Ubuntu 22.04 LTS (or WSL2 on Windows) with Python 3.10+
- Familiarity with command-line interfaces

No prior robotics or ROS experience is required.

## Concepts

### What is Physical AI?

Physical AI represents the marriage of artificial intelligence with physical embodiment. While traditional AI systems process information and generate outputs in digital space (like chatbots, recommendation systems, or image recognition), Physical AI systems exist in the real world and must navigate the complexities of three-dimensional space, physics, and real-time interactions.

Key characteristics of Physical AI systems:
- **Embodiment**: They have a physical form that interacts with the environment
- **Real-time constraints**: They must respond to environmental changes within strict time limits
- **Sensorimotor integration**: They combine sensory input with motor output in continuous loops
- **Uncertainty management**: They operate in unpredictable environments with noisy sensors

### The Sense-Think-Act Loop

At the heart of every robot lies the fundamental sense-think-act loop:

1. **Sense**: Collect information about the environment through sensors (cameras, LiDAR, IMUs, tactile sensors)
2. **Think**: Process sensory data through algorithms to understand the state and plan actions
3. **Act**: Execute physical actions through actuators (motors, grippers, displays)

This loop runs continuously, with each iteration building on the previous one. For example, a delivery robot might sense obstacles in its path, think about alternative routes, and act by adjusting its direction.

### Why Robots Need Middleware

Unlike traditional applications that run on a single computer, robotic systems are inherently distributed. A humanoid robot might have:
- Sensor processing running on a GPU
- Path planning on a main CPU
- Motor control on microcontrollers
- High-level AI on cloud services

This distributed nature creates complex communication challenges. Robots need middleware that handles:
- Message passing between processes and machines
- Device abstraction (hiding hardware details)
- Real-time performance guarantees
- Fault tolerance and recovery

### Introduction to ROS 2

ROS 2 (Robot Operating System 2) is the industry-standard middleware for robotics development. Despite its name, ROS 2 is not an operating system but rather a collection of libraries, tools, and conventions that simplify distributed robotic applications.

Key features of ROS 2:
- **Distributed communication**: Nodes communicate via topics (pub/sub) and services (request/reply)
- **Language support**: C++, Python, and other languages can interoperate
- **Real-time capabilities**: Support for time-sensitive applications
- **Security**: Authentication, encryption, and access control
- **Middleware agnostic**: Can work with different communication layers (DDS implementations)

## Examples

### Example 1: Installing ROS 2 Humble Hawksbill

First, let's install ROS 2 Humble Hawksbill, which is an LTS (Long Term Support) version suitable for production applications.

```bash
# Add ROS 2 apt repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y python3-rosdep2 python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

Set up your environment:
```bash
# Add to your ~/.bashrc to source ROS 2 environment automatically
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Verify the installation:
```bash
# Check ROS 2 version
ros2 --version

# Run a simple demo
ros2 run demo_nodes_cpp talker
```

### Example 2: Creating Your First ROS 2 Publisher Node

Let's create a workspace and build a simple publisher node that broadcasts messages:

```bash
# Create a workspace
mkdir -p ~/physical_ai_ws/src
cd ~/physical_ai_ws

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Create a package for our first robot node
cd src
ros2 pkg create --build-type ament_python robot_basics
cd robot_basics
```

Create the publisher node file `robot_basics/robot_basics/talker_node.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TalkerNode(Node):
    """
    A simple ROS 2 publisher node that sends messages.
    Demonstrates the basic structure of a ROS 2 node.
    """

    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, 'robot_messages', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello Physical AI! Count: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    talker = TalkerNode()

    try:
        rclpy.spin(talker)
    except KeyboardInterrupt:
        pass
    finally:
        talker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Example 3: Creating a Subscriber Node

Create a subscriber to receive messages from the publisher. Add `robot_basics/robot_basics/listener_node.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ListenerNode(Node):
    """
    A simple ROS 2 subscriber node that receives messages.
    Works in conjunction with the talker node.
    """

    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'robot_messages',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    listener = ListenerNode()

    try:
        rclpy.spin(listener)
    except KeyboardInterrupt:
        pass
    finally:
        listener.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Example 4: Visualizing the Computation Graph

Run both nodes and visualize their communication:

Terminal 1 - Run the talker:
```bash
cd ~/physical_ai_ws
source install/setup.bash
ros2 run robot_basics talker
```

Terminal 2 - Run the listener:
```bash
cd ~/physical_ai_ws
source install/setup.bash
ros2 run robot_basics listener
```

Terminal 3 - Visualize the graph:
```bash
# Install rqt if not already installed
sudo apt install ros-humble-rqt ros-humble-rqt-graph

# Run the graph visualizer
rqt_graph
```

This will open a GUI showing the talker node publishing to the robot_messages topic and the listener node subscribed to it.

## Exercise

### Required Exercise: Your First Robot Communication System

Build a simple sensor fusion system that demonstrates the sense-think-act loop in ROS 2:

**Task**: Create a system with three nodes:
1. `temperature_sensor` - Publishes simulated temperature readings
2. `humidity_sensor` - Publishes simulated humidity readings
3. `climate_fusion` - Subscribes to both sensors and publishes combined climate data

**Acceptance Criteria**:
- All three nodes successfully communicate using ROS 2 topics
- The climate_fusion node correctly combines data from both sensors
- The system runs without errors for at least 30 seconds
- You can visualize the node connections using `rqt_graph`

**Implementation Steps**:
1. Create a new package called `climate_monitor`
2. Implement the three nodes as described above
3. Use `sensor_msgs/msg/Temperature` and `sensor_msgs/msg/RelativeHumidity` message types
4. Create a custom message for combined climate data
5. Test the system and verify communication

**Extension Challenges**:
- **Beginner**: Add a simple visualization node that prints climate data in a formatted way
- **Intermediate**: Implement a climate alert system that publishes warnings when thresholds are exceeded
- **Advanced**: Add a service that allows external nodes to request current climate summary on demand

## References

- [ROS 2 Documentation - Humble Hawksbill](https://docs.ros.org/en/humble/)
- [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [ROS 2 Concepts](https://docs.ros.org/en/humble/Concepts.html)
- [Physical AI Research at Stanford](https://paikit.stanford.edu/)
- [Robotics Middleware Comparison](https://www.researchgate.net/publication/342010123_Robotics_Middleware_Comparison)

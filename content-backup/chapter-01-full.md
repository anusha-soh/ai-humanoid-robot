# Chapter 1: Welcome to Physical AI

## Overview

**Learning Objectives:**
- Understand what Physical AI means and why it matters
- Learn the sense-think-act loop that powers all robots
- Install ROS 2 and run your first robot program
- Visualize how robots communicate using computation graphs

**Prerequisites:**
- Basic Python knowledge (variables, functions, loops)
- Familiarity with command line (cd, ls, running commands)
- Ubuntu 22.04 LTS or WSL2 with Ubuntu

**Time to Complete:** 2-3 hours

---

## Introduction

Welcome to the world of Physical AI! You're about to learn how to build intelligent robots that can see, think, and act in the real world.

**What is Physical AI?** It combines AI with physical systems. Chatbots and image tools only process data. Physical AI systems move, grab objects, and navigate rooms. Think self-driving cars, warehouse robots, and humanoid helpers.

This book teaches you to build these systems from scratch. You'll start with basics (this chapter!). Then you'll create a humanoid butler. It will understand voice commands, move through rooms, and grab objects.

**Why ROS 2?** Robot Operating System 2 (ROS 2) is the standard tool for robotics. Companies like BMW, NASA, and Boston Dynamics use it. You'll learn ROS 2 because it handles the hard parts: how sensors talk to motors, keeping time in sync, and running many programs at once.

---

## The Sense-Think-Act Loop

Every robot follows the same basic pattern:

1. **Sense**: Gather data from the world (cameras, LiDAR, touch sensors)
2. **Think**: Process that data to make decisions (AI models, planning algorithms)
3. **Act**: Execute actions based on decisions (move motors, speak, grasp objects)

Then repeat this loop continuously, often hundreds of times per second.

**Example: A humanoid butler fetching a cup**

- **Sense**: Camera detects a red cup on the table
- **Think**: AI identifies "cup", plans path to table, calculates grasp
- **Act**: Robot walks to table, extends arm, closes gripper around cup

This loop is fundamental. You'll implement it in every chapter of this book.

### Why Robots Need Middleware

Imagine building a robot from scratch:
- Your camera outputs images 30 times per second
- Your wheels need speed commands 100 times per second
- Your AI model takes 50 milliseconds to process each image
- Your navigation needs your location all the time

How do you coordinate all of this? How do you get the latest camera image to your AI? How do you keep the wheels moving while AI thinks?

**Middleware solves this.** ROS 2 provides:
- **Message passing**: Parts send data to each other easily
- **Time sync**: All parts use the same clock
- **Process control**: Start and watch many programs at once
- **Standard formats**: Cameras, motors, and sensors speak the same language

### ROS 2 Overview

ROS 2 is not an operating system (despite the name). It's a set of tools that help you build robot software.

**Key concepts:**
- **Nodes**: Separate programs (one for camera, one for AI, one for motors)
- **Topics**: Channels nodes use to send messages (like radio channels)
- **Messages**: Data packets (sensor readings, commands, images)
- **Services**: Ask-and-answer talks (ask for data, get a reply)

You'll learn each concept hands-on in the coming chapters. For now, let's install ROS 2 and run your first program.

---

## Code Examples

### Example 1: Install ROS 2 and Verify Installation

First, install ROS 2 Humble (the Long-Term Support version).

**Open a terminal and run these commands:**

```bash
# Set up sources
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

# Add ROS 2 repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS 2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop -y

# Install development tools
sudo apt install python3-colcon-common-extensions python3-rosdep -y
```

**Verify the installation:**

```bash
# Source ROS 2 environment (do this every time you open a new terminal)
source /opt/ros/humble/setup.bash

# Check ROS 2 version
ros2 --version
```

**Expected Output:**
```
ros2 cli version 0.25.x
```

**Make it permanent** (so you don't have to source every time):

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Explanation:**
- `apt install ros-humble-desktop`: Installs ROS 2 with visual tools
- `source /opt/ros/humble/setup.bash`: Loads ROS 2 into your system
- `ros2 --version`: Checks that ROS 2 is installed

---

### Example 2: Run a "Hello World" ROS 2 Node

ROS 2 comes with demo programs. Let's run a simple talker node that publishes messages.

**Run the demo talker:**

```bash
ros2 run demo_nodes_cpp talker
```

**Expected Output:**
```
[INFO] [1702345678.123456789] [talker]: Publishing: 'Hello World: 0'
[INFO] [1702345679.123456789] [talker]: Publishing: 'Hello World: 1'
[INFO] [1702345680.123456789] [talker]: Publishing: 'Hello World: 2'
...
```

The talker publishes messages to a topic. Press `Ctrl+C` to stop it.

**Run the demo listener** (in a new terminal):

```bash
# Terminal 2
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp listener
```

**Expected Output:**
```
[INFO] [1702345678.456789012] [listener]: I heard: 'Hello World: 0'
[INFO] [1702345679.456789012] [listener]: I heard: 'Hello World: 1'
...
```

The listener subscribes to the same topic and receives the messages!

**Explanation:**
- `ros2 run <package> <executable>`: Runs a program from a ROS 2 package
- `talker` publishes to the `/chatter` topic
- `listener` subscribes to `/chatter` and prints received messages
- This demonstrates **publish-subscribe communication** in ROS 2

---

### Example 3: Visualize the Computation Graph with rqt_graph

ROS 2 provides tools to visualize how nodes communicate.

**Run rqt_graph** (while talker and listener are still running):

```bash
# Terminal 3
source /opt/ros/humble/setup.bash
ros2 run rqt_graph rqt_graph
```

A window opens showing:
- Two boxes: `/talker` and `/listener` (the nodes)
- An arrow labeled `/chatter` connecting them (the topic)

This visualization shows the **computation graph** - how data flows through your robot system.

**Try this:** Stop the listener (Ctrl+C in Terminal 2) and refresh rqt_graph. The `/listener` node disappears, but `/talker` keeps publishing. This shows nodes are independent - one can crash without affecting others.

**Explanation:**
- `rqt_graph`: Visual debugging tool for ROS 2 systems
- Shows real-time node and topic relationships
- Essential for debugging complex robot systems with many nodes

---

## Exercise

### Required Exercise: Create Your First ROS 2 Publisher

**Goal:** Write a Python program that publishes "Hello, Physical AI!" messages to a custom topic.

**Acceptance Criteria:**
- [ ] Creates a ROS 2 node named `physical_ai_talker`
- [ ] Publishes string messages to `/physical_ai_greetings` topic
- [ ] Messages say "Hello, Physical AI! Count: X" (where X increments)
- [ ] Publishes at 1 Hz (once per second)
- [ ] Visible in `rqt_graph`

**Instructions:**

1. Create a workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Create a Python file `physical_ai_talker.py`:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PhysicalAITalker(Node):
    def __init__(self):
        super().__init__('physical_ai_talker')
        self.publisher = self.create_publisher(String, '/physical_ai_greetings', 10)
        self.timer = self.create_timer(1.0, self.publish_message)
        self.count = 0

    def publish_message(self):
        msg = String()
        msg.data = f'Hello, Physical AI! Count: {self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = PhysicalAITalker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

3. Run your node:
```bash
cd ~/ros2_ws
python3 src/physical_ai_talker.py
```

4. In another terminal, listen to your topic:
```bash
ros2 topic echo /physical_ai_greetings
```

5. Verify in `rqt_graph` that your node appears!

**Hints:**
- `rclpy.init()`: Initializes ROS 2 for Python
- `create_publisher()`: Creates a publisher on a topic
- `create_timer()`: Calls a function repeatedly at a fixed rate
- `rclpy.spin()`: Keeps the node running until Ctrl+C

**Solution** is provided in the appendix if you get stuck.

---

### Extension Exercise - Beginner: Add a Subscriber

Modify your program to also listen to the standard `/chatter` topic and print received messages.

**Hint:** Use `create_subscription()` similar to how you used `create_publisher()`.

---

### Extension Exercise - Intermediate: Custom Message Counter

Instead of a string, publish a custom count as an integer message type. Use `std_msgs/msg/Int32`.

---

### Extension Exercise - Advanced: Bidirectional Communication

Create a system with two nodes:
- Node A publishes greetings
- Node B subscribes to greetings and replies with acknowledgments
- Node A subscribes to acknowledgments and prints them

---

## References

### Official Documentation
- [ROS 2 Documentation](https://docs.ros.org/en/humble/) - Official ROS 2 guide
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html) - Step-by-step tutorials
- [rclpy API Reference](https://docs.ros.org/en/humble/p/rclpy/) - Python client library docs

### Further Reading
- [Understanding ROS 2 Nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)
- [ROS 2 Topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
- [Why ROS 2?](https://design.ros2.org/articles/why_ros2.html) - Design decisions behind ROS 2

### Related Chapters
- Chapter 2: ROS 2 Architecture - Dive deeper into nodes, topics, and services
- Chapter 4: Python + ROS 2 - Build complex AI-driven nodes with rclpy

---

## Summary

**What You Learned:**
- Physical AI combines intelligence with physical systems (robots that move and act)
- The sense-think-act loop is the foundation of all robot behavior
- ROS 2 is middleware that coordinates robot components
- Nodes are independent programs that communicate via topics
- You can visualize systems with `rqt_graph`

**Key Commands:**
```bash
source /opt/ros/humble/setup.bash  # Load ROS 2 environment
ros2 run <package> <node>          # Run a ROS 2 program
ros2 topic echo <topic>            # Listen to messages on a topic
ros2 run rqt_graph rqt_graph       # Visualize computation graph
```

**Next Steps:**
In Chapter 2, you'll learn how nodes, topics, and services work in detail. You'll build multi-node systems with sensor fusion and understand the ROS 2 computation graph deeply.

---

**Navigation:**
- ← Previous: [Introduction](../intro)
- → Next: [Chapter 2: ROS 2 Architecture](./chapter-02)

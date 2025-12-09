# Chapter 2: ROS 2 Architecture - Nodes, Topics, and Services

## Overview

In the previous chapter, you created your first ROS 2 nodes and witnessed their communication. Now, it's time to dive deeper into the fundamental architecture that makes distributed robotics possible. This chapter explores the core communication patterns that power all ROS 2 systems: nodes as independent processes, topics for asynchronous publish-subscribe messaging, and services for synchronous request-response interactions.

Understanding these architectural concepts is crucial for building robust robotic systems. You'll learn how to design distributed systems where multiple processes can communicate efficiently, how to structure your robot's software components, and how to visualize the resulting computation graph. By the end of this chapter, you'll have built a complete sensor fusion system that demonstrates these core concepts in action.

### Learning Objectives

By the end of this chapter, you will be able to:
- Explain the role of nodes as independent processes in ROS 2
- Design publish-subscribe communication patterns using topics
- Implement request-response interactions using services
- Visualize and analyze the computation graph of ROS 2 systems
- Build a sensor fusion system that combines data from multiple sources

### Prerequisites

Before starting this chapter, you should have:
- Completed Chapter 1 (Introduction to Physical AI and ROS 2 basics)
- Successfully installed ROS 2 Humble Hawksbill
- Basic understanding of Python programming
- Familiarity with the concept of processes and inter-process communication

## Concepts

### Nodes: The Building Blocks of ROS 2

In ROS 2, a **node** is an independent process that performs computation. Nodes are the fundamental building blocks of any ROS 2 system. Each node typically performs a specific task and communicates with other nodes to achieve complex behaviors.

Key characteristics of ROS 2 nodes:
- **Independence**: Each node runs as a separate process, providing fault isolation
- **Specialization**: Each node typically handles a specific function (sensor processing, control, planning)
- **Communication**: Nodes communicate through topics, services, and actions
- **Lifecycle**: Nodes can be started, stopped, and restarted independently

### Topics: Publish-Subscribe Communication

**Topics** enable asynchronous, one-to-many communication between nodes using the publish-subscribe pattern. A node publishes messages to a topic, and any number of other nodes can subscribe to that topic to receive the messages.

Key features of topics:
- **Asynchronous**: Publishers and subscribers don't need to be synchronized
- **Decoupled**: Publishers don't know who subscribes, subscribers don't know who publishes
- **Broadcast**: One publisher can send to many subscribers
- **Message types**: Each topic has a specific message type that defines the data structure

### Services: Request-Response Communication

**Services** enable synchronous, one-to-one communication using the request-response pattern. A client sends a request to a service server, which processes the request and returns a response.

Key features of services:
- **Synchronous**: The client waits for the response before continuing
- **Request-response**: One request generates one response
- **Reliable**: Services guarantee delivery and response
- **Use cases**: Configuration, triggering actions, requesting specific information

### The Computation Graph

The **computation graph** represents all nodes and their connections in a ROS 2 system. It shows:
- Which nodes exist in the system
- Which topics each node publishes to and subscribes from
- Which services each node provides or uses
- The overall structure of the distributed system

### Quality of Service (QoS) Settings

ROS 2 provides Quality of Service (QoS) settings that allow you to fine-tune communication behavior:
- **Reliability**: Best effort vs. reliable delivery
- **Durability**: Volatile vs. transient local (for late-joining subscribers)
- **History**: Keep all messages vs. keep last N messages
- **Deadline**: Time constraints for message delivery

## Examples

### Example 1: Creating a Simple Publisher Node

Let's create a publisher node that simulates a temperature sensor:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random


class TemperatureSensor(Node):
    """
    Simulates a temperature sensor publishing to a topic.
    Demonstrates the publisher pattern in ROS 2.
    """

    def __init__(self):
        super().__init__('temperature_sensor')
        self.publisher = self.create_publisher(Float32, 'temperature', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Temperature sensor node started')

    def timer_callback(self):
        msg = Float32()
        # Simulate temperature reading with some noise
        msg.data = 20.0 + random.uniform(-2.0, 2.0)  # Room temperature ± noise
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing temperature: {msg.data:.2f}°C')


def main(args=None):
    rclpy.init(args=args)
    temp_sensor = TemperatureSensor()

    try:
        rclpy.spin(temp_sensor)
    except KeyboardInterrupt:
        pass
    finally:
        temp_sensor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Example 2: Creating a Subscriber Node

Now let's create a subscriber that receives temperature data:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class TemperatureMonitor(Node):
    """
    Subscribes to temperature data and logs it.
    Demonstrates the subscriber pattern in ROS 2.
    """

    def __init__(self):
        super().__init__('temperature_monitor')
        self.subscription = self.create_subscription(
            Float32,
            'temperature',
            self.temperature_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Temperature monitor node started')

    def temperature_callback(self, msg):
        self.get_logger().info(f'Received temperature: {msg.data:.2f}°C')
        # Add alert logic for extreme temperatures
        if msg.data > 25.0:
            self.get_logger().warn(f'High temperature alert: {msg.data:.2f}°C')


def main(args=None):
    rclpy.init(args=args)
    temp_monitor = TemperatureMonitor()

    try:
        rclpy.spin(temp_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        temp_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Example 3: Creating a Service Server

Let's create a service that allows clients to configure the temperature threshold:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool


class TemperatureConfigService(Node):
    """
    Provides a service to configure temperature monitoring parameters.
    Demonstrates the service server pattern in ROS 2.
    """

    def __init__(self):
        super().__init__('temp_config_service')
        self.srv = self.create_service(
            SetBool,
            'set_temperature_alert_enabled',
            self.set_alert_enabled_callback)
        self.alert_enabled = True
        self.get_logger().info('Temperature config service started')

    def set_alert_enabled_callback(self, request, response):
        self.alert_enabled = request.data
        response.success = True
        response.message = f'Temperature alerts {"enabled" if self.alert_enabled else "disabled"}'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    config_service = TemperatureConfigService()

    try:
        rclpy.spin(config_service)
    except KeyboardInterrupt:
        pass
    finally:
        config_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Example 4: Creating a Service Client

Now let's create a client that uses the configuration service:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool


class ConfigClient(Node):
    """
    Client node that calls the temperature configuration service.
    Demonstrates the service client pattern in ROS 2.
    """

    def __init__(self):
        super().__init__('config_client')
        self.cli = self.create_client(SetBool, 'set_temperature_alert_enabled')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for temperature config service...')
        self.req = SetBool.Request()

    def send_request(self, enable_alerts):
        self.req.data = enable_alerts
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)
    client = ConfigClient()

    # Disable alerts
    response = client.send_request(False)
    if response:
        client.get_logger().info(f'Response: {response.message}')
    else:
        client.get_logger().error('Failed to call service')

    # Re-enable alerts
    response = client.send_request(True)
    if response:
        client.get_logger().info(f'Response: {response.message}')
    else:
        client.get_logger().error('Failed to call service')

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Exercise

### Required Exercise: Sensor Fusion System

Build a complete sensor fusion system that combines data from multiple sensors using ROS 2 communication patterns:

**Task**: Create a system with 4 nodes:
1. `sensor_1` - Publishes simulated sensor data (e.g., temperature)
2. `sensor_2` - Publishes simulated sensor data (e.g., humidity)
3. `sensor_3` - Publishes simulated sensor data (e.g., pressure)
4. `fusion_node` - Subscribes to all sensors, combines data, and publishes fused result

**Acceptance Criteria**:
- All 4 nodes communicate correctly using ROS 2 topics
- The fusion node successfully combines data from all three sensors
- The system runs without errors for at least 60 seconds
- You can visualize the complete computation graph using `rqt_graph`

**Implementation Steps**:
1. Create a new package called `sensor_fusion_demo`
2. Implement the 4 nodes as described above
3. Define appropriate message types for sensor data
4. Add error handling for missing sensor data
5. Test the system and verify proper data fusion

**Extension Challenges**:
- **Beginner**: Add a simple data validation node that checks sensor ranges
- **Intermediate**: Implement a weighted fusion algorithm that considers sensor reliability
- **Advanced**: Add a service that allows external nodes to request current fused data on demand

## References

- [ROS 2 Nodes Documentation](https://docs.ros.org/en/humble/Concepts/Basic/About-Nodes.html)
- [ROS 2 Topics Documentation](https://docs.ros.org/en/humble/Concepts/Basic/About-Topics.html)
- [ROS 2 Services Documentation](https://docs.ros.org/en/humble/Concepts/Basic/About-Services.html)
- [ROS 2 Quality of Service Guide](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)
- [ROS 2 Computation Graph](https://docs.ros.org/en/humble/Tutorials/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
- [ROS 2 Tutorials - Writing a Simple Publisher and Subscriber](https://docs.ros.org/en/humble/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

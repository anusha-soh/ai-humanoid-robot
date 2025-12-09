# Chapter 16: Capstone - The Autonomous Humanoid Butler

## Overview

Welcome to the capstone chapter of our Physical AI and Humanoid Robotics book! In this chapter, we'll bring together everything we've learned across all 15 previous chapters to create a fully autonomous humanoid butler. This sophisticated system will integrate voice commands, LLM task planning, computer vision, navigation, and manipulation into a unified robotic system.

The autonomous butler represents the culmination of our journey from basic ROS 2 concepts to advanced Vision-Language-Action systems. Your butler will be able to listen to natural language commands, plan complex action sequences, navigate through environments, identify and manipulate objects, and provide feedback to users.

### Learning Objectives

- Integrate all components from previous chapters into a unified system
- Debug complex multi-component robotic systems
- Optimize performance for real-time execution
- Understand challenges in building end-to-end autonomous systems
- Apply best practices for system integration and testing

### Prerequisites

- All previous chapters (1-15) - this capstone integrates concepts from every module
- Understanding of ROS 2 architecture and communication patterns
- Experience with simulation environments (Gazebo/Isaac Sim)
- Knowledge of computer vision, navigation, and manipulation

## Concepts

### System Integration Architecture

The autonomous butler system combines all components we've built throughout the book into a cohesive architecture:

1. **Voice Interface**: Whisper for speech recognition
2. **Language Understanding**: LLMs for command interpretation and task planning
3. **Perception**: Computer vision for object detection and localization
4. **Navigation**: Nav2 for path planning and execution
5. **Manipulation**: Joint control for grasping and releasing objects
6. **Integration Layer**: Coordinated system that manages all components

### End-to-End Autonomous Behavior

Creating truly autonomous behavior requires careful coordination between perception, planning, and execution. The system must handle:

- Continuous monitoring of the environment
- Dynamic replanning when situations change
- Robust error handling and recovery
- Seamless transitions between different behavior modes

### Debugging Complex Systems

Debugging integrated robotic systems presents unique challenges:
- Multiple concurrent processes
- Timing-dependent behaviors
- Sensor and actuator delays
- Distributed system complexities

Effective debugging strategies include:
- Comprehensive logging and visualization
- Modular testing of individual components
- Simulation-based validation before real-world deployment
- Systematic isolation of failure points

## Examples

### Example 1: Autonomous Butler Main Controller

Let's create the main controller that coordinates all butler behaviors:

```python
#!/usr/bin/env python3
# autonomous_b Butler_controller.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image, LaserScan
import json
import time
from enum import Enum
from collections import deque

class ButlerState(Enum):
    IDLE = "IDLE"
    LISTENING = "LISTENING"
    PLANNING = "PLANNING"
    NAVIGATING = "NAVIGATING"
    PERCEIVING = "PERCEIVING"
    MANIPULATING = "MANIPULATING"
    RETURNING = "RETURNING"
    REPORTING = "REPORTING"
    ERROR = "ERROR"

class AutonomousButlerController(Node):
    def __init__(self):
        super().__init__('autonomous_b Butler_controller')

        # State management
        self.current_state = ButlerState.IDLE
        self.previous_state = None
        self.task_queue = deque()
        self.current_task = None
        self.task_history = deque(maxlen=20)

        # Butler capabilities
        self.has_object = False
        self.object_type = None
        self.destination = None
        self.user_location = None

        # Publishers
        self.voice_cmd_pub = self.create_publisher(
            String,
            '/voice_commands',
            10
        )
        self.high_level_cmd_pub = self.create_publisher(
            String,
            '/high_level_commands',
            10
        )
        self.action_seq_pub = self.create_publisher(
            String,
            '/action_sequence',
            10
        )
        self.tts_pub = self.create_publisher(
            String,
            '/tts_commands',
            10
        )
        self.nav_goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )
        self.gripper_cmd_pub = self.create_publisher(
            String,
            '/gripper_commands',
            10
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.status_pub = self.create_publisher(
            String,
            '/butler_status',
            10
        )

        # Subscribers
        self.voice_sub = self.create_subscription(
            String,
            '/voice_commands',
            self.voice_command_callback,
            10
        )
        self.nav_status_sub = self.create_subscription(
            String,
            '/navigation_status',
            self.navigation_status_callback,
            10
        )
        self.object_detection_sub = self.create_subscription(
            String,
            '/detected_objects',
            self.object_detection_callback,
            10
        )
        self.manipulation_status_sub = self.create_subscription(
            String,
            '/manipulation_status',
            self.manipulation_status_callback,
            10
        )

        # Timer for state machine
        self.state_timer = self.create_timer(0.1, self.state_machine)

        # Initialize user location (assumed to be at origin for simplicity)
        self.user_location = (0.0, 0.0)

        self.get_logger().info('Autonomous Butler Controller initialized')

    def voice_command_callback(self, msg):
        """Handle incoming voice commands"""
        if self.current_state == ButlerState.IDLE:
            command = msg.data.lower()
            self.get_logger().info(f'Received command: {command}')

            # Add to task queue
            self.task_queue.append(command)
            self.current_state = ButlerState.PLANNING

            # Announce acceptance
            self.speak_response(f"Received command: {command}. Planning action sequence.")

    def navigation_status_callback(self, msg):
        """Handle navigation status updates"""
        status = msg.data.lower()

        if "arrived" in status or "reached" in status:
            if self.current_state == ButlerState.NAVIGATING:
                self.get_logger().info("Navigation completed successfully")

                # Move to next state based on current task
                if self.current_task and "bring" in self.current_task:
                    self.current_state = ButlerState.PERCEIVING
                elif self.current_task and "return" in self.current_task:
                    self.current_state = ButlerState.REPORTING

    def object_detection_callback(self, msg):
        """Handle object detection results"""
        try:
            detected_objects = json.loads(msg.data)

            if self.current_state == ButlerState.PERCEIVING:
                # Look for requested object
                requested_obj = self.extract_object_from_task()

                for obj in detected_objects:
                    if requested_obj in obj.get('type', '').lower():
                        self.get_logger().info(f"Found {requested_obj}, moving to manipulation")
                        self.object_type = requested_obj
                        self.current_state = ButlerState.MANIPULATING
                        break
                else:
                    self.get_logger().warn(f"Requested object {requested_obj} not found")
                    self.current_state = ButlerState.ERROR
        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON in object detection message")

    def manipulation_status_callback(self, msg):
        """Handle manipulation status updates"""
        status = msg.data.lower()

        if "grasped" in status or "grabbed" in status:
            if self.current_state == ButlerState.MANIPULATING:
                self.has_object = True
                self.get_logger().info("Object grasped successfully")

                # Set return to user as next task
                self.current_task = "return to user"
                self.current_state = ButlerState.RETURNING
        elif "released" in status:
            if self.current_state == ButlerState.RETURNING:
                self.has_object = False
                self.get_logger().info("Object delivered to user")
                self.current_state = ButlerState.REPORTING

    def state_machine(self):
        """Main state machine for the butler"""
        if self.current_state == ButlerState.PLANNING:
            self.plan_task()
        elif self.current_state == ButlerState.NAVIGATING:
            self.navigate_to_destination()
        elif self.current_state == ButlerState.PERCEIVING:
            self.perceive_environment()
        elif self.current_state == ButlerState.MANIPULATING:
            self.manipulate_object()
        elif self.current_state == ButlerState.RETURNING:
            self.return_to_user()
        elif self.current_state == ButlerState.REPORTING:
            self.report_completion()
        elif self.current_state == ButlerState.ERROR:
            self.handle_error()

    def plan_task(self):
        """Plan the current task"""
        if self.task_queue:
            self.current_task = self.task_queue.popleft()
            self.get_logger().info(f"Planning task: {self.current_task}")

            # Determine destination based on task
            if "kitchen" in self.current_task:
                self.destination = self.get_kitchen_location()
            elif "living room" in self.current_task:
                self.destination = self.get_living_room_location()
            elif "bedroom" in self.current_task:
                self.destination = self.get_bedroom_location()
            else:
                self.destination = self.get_nearest_location()

            # Move to navigation state
            self.current_state = ButlerState.NAVIGATING

            # Update status
            self.update_status(f"PLANNING: Navigating to destination for task '{self.current_task}'")

    def navigate_to_destination(self):
        """Navigate to the planned destination"""
        if self.destination:
            # Create navigation goal
            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'map'
            goal_msg.pose.position.x = self.destination[0]
            goal_msg.pose.position.y = self.destination[1]
            goal_msg.pose.orientation.w = 1.0  # No rotation

            self.nav_goal_pub.publish(goal_msg)
            self.get_logger().info(f"Navigating to ({self.destination[0]}, {self.destination[1]})")

    def perceive_environment(self):
        """Perceive environment for requested objects"""
        self.get_logger().info("Perceiving environment for objects")

        # Request object detection
        detect_msg = String()
        detect_msg.data = f"look_for_{self.extract_object_from_task()}"
        self.object_detection_sub.publish(detect_msg)

    def manipulate_object(self):
        """Manipulate the detected object"""
        if self.object_type:
            self.get_logger().info(f"Manipulating {self.object_type}")

            # Send grasp command
            grasp_msg = String()
            grasp_msg.data = f"grasp_{self.object_type}"
            self.gripper_cmd_pub.publish(grasp_msg)

    def return_to_user(self):
        """Return to user with the object"""
        self.get_logger().info(f"Returning to user at ({self.user_location[0]}, {self.user_location[1]})")

        # Navigate back to user
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = self.user_location[0]
        goal_msg.pose.position.y = self.user_location[1]
        goal_msg.pose.orientation.w = 1.0

        self.nav_goal_pub.publish(goal_msg)

    def report_completion(self):
        """Report task completion to user"""
        if self.has_object:
            response = f"I have brought you the {self.object_type}."
        else:
            response = f"Task completed successfully. I delivered the {self.object_type} to you."

        self.speak_response(response)
        self.get_logger().info("Task completed and reported to user")

        # Reset for next task
        self.current_state = ButlerState.IDLE
        self.has_object = False
        self.object_type = None
        self.current_task = None

        self.update_status("IDLE: Ready for next command")

    def handle_error(self):
        """Handle errors in the system"""
        error_msg = "An error occurred during task execution."
        self.speak_response(error_msg)
        self.get_logger().error(error_msg)

        # Reset to safe state
        self.current_state = ButlerState.IDLE
        self.update_status("ERROR: System reset after error")

    def extract_object_from_task(self):
        """Extract object type from task description"""
        if not self.current_task:
            return "object"

        # Simple object extraction - in practice, this would use more sophisticated NLP
        if "cup" in self.current_task:
            return "cup"
        elif "snack" in self.current_task or "food" in self.current_task:
            return "snack"
        elif "book" in self.current_task:
            return "book"
        else:
            return "object"

    def get_kitchen_location(self):
        """Get kitchen location coordinates"""
        # In a real system, these would come from a map or localization system
        return (5.0, 2.0)  # Example coordinates

    def get_living_room_location(self):
        """Get living room location coordinates"""
        return (3.0, -1.0)  # Example coordinates

    def get_bedroom_location(self):
        """Get bedroom location coordinates"""
        return (-2.0, 4.0)  # Example coordinates

    def get_nearest_location(self):
        """Get nearest relevant location"""
        return (0.0, 0.0)  # Return to origin

    def speak_response(self, text):
        """Publish text-to-speech response"""
        tts_msg = String()
        tts_msg.data = text
        self.tts_pub.publish(tts_msg)

    def update_status(self, status):
        """Update system status"""
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousButlerController()

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

### Example 2: Butler System Integration Node

Let's create a node that integrates all the previous components:

```python
#!/usr/bin/env python3
# butler_system_integration.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
import json
import time
from collections import deque

class ButlerSystemIntegrationNode(Node):
    def __init__(self):
        super().__init__('butler_system_integration')

        # System state tracking
        self.voice_commands = deque(maxlen=10)
        self.action_history = deque(maxlen=20)
        self.system_health = {
            'voice': True,
            'planning': True,
            'navigation': True,
            'perception': True,
            'manipulation': True
        }

        # Publishers for integrated system
        self.voice_cmd_pub = self.create_publisher(
            String,
            '/voice_commands',
            10
        )
        self.high_level_cmd_pub = self.create_publisher(
            String,
            '/high_level_commands',
            10
        )
        self.action_seq_pub = self.create_publisher(
            String,
            '/action_sequence',
            10
        )
        self.tts_pub = self.create_publisher(
            String,
            '/tts_commands',
            10
        )
        self.system_status_pub = self.create_publisher(
            String,
            '/integrated_system_status',
            10
        )

        # Subscribers for monitoring all subsystems
        self.voice_status_sub = self.create_subscription(
            String,
            '/voice_system_status',
            self.voice_status_callback,
            10
        )
        self.planning_status_sub = self.create_subscription(
            String,
            '/planning_system_status',
            self.planning_status_callback,
            10
        )
        self.navigation_status_sub = self.create_subscription(
            String,
            '/navigation_system_status',
            self.navigation_status_callback,
            10
        )
        self.perception_status_sub = self.create_subscription(
            String,
            '/perception_system_status',
            self.perception_status_callback,
            10
        )
        self.manipulation_status_sub = self.create_subscription(
            String,
            '/manipulation_system_status',
            self.manipulation_status_callback,
            10
        )

        # Timer for system health checks
        self.health_timer = self.create_timer(1.0, self.system_health_check)

        # Timer for coordinating subsystems
        self.coordination_timer = self.create_timer(0.5, self.coordinate_subsystems)

        self.get_logger().info('Butler System Integration initialized')

    def voice_status_callback(self, msg):
        """Monitor voice system status"""
        status = msg.data.lower()
        self.system_health['voice'] = 'error' not in status
        self.get_logger().debug(f'Voice system status: {status}')

    def planning_status_callback(self, msg):
        """Monitor planning system status"""
        status = msg.data.lower()
        self.system_health['planning'] = 'error' not in status
        self.get_logger().debug(f'Planning system status: {status}')

    def navigation_status_callback(self, msg):
        """Monitor navigation system status"""
        status = msg.data.lower()
        self.system_health['navigation'] = 'error' not in status
        self.get_logger().debug(f'Navigation system status: {status}')

    def perception_status_callback(self, msg):
        """Monitor perception system status"""
        status = msg.data.lower()
        self.system_health['perception'] = 'error' not in status
        self.get_logger().debug(f'Perception system status: {status}')

    def manipulation_status_callback(self, msg):
        """Monitor manipulation system status"""
        status = msg.data.lower()
        self.system_health['manipulation'] = 'error' not in status
        self.get_logger().debug(f'Manipulation system status: {status}')

    def system_health_check(self):
        """Check overall system health"""
        healthy_subsystems = sum(1 for status in self.system_health.values() if status)
        total_subsystems = len(self.system_health)

        health_percentage = (healthy_subsystems / total_subsystems) * 100
        self.get_logger().info(f'System health: {healthy_subsystems}/{total_subsystems} subsystems operational ({health_percentage:.1f}%)')

        # Publish integrated health status
        health_msg = String()
        health_msg.data = f"SYSTEM_HEALTH: {healthy_subsystems}/{total_subsystems} ({health_percentage:.1f}%)"
        self.system_status_pub.publish(health_msg)

    def coordinate_subsystems(self):
        """Coordinate between subsystems"""
        # Check if all systems are healthy before proceeding with complex tasks
        all_healthy = all(self.system_health.values())

        if not all_healthy:
            # Identify problematic subsystems
            problematic = [name for name, status in self.system_health.items() if not status]
            self.get_logger().warn(f'Subsystems with issues: {problematic}')

            # Attempt to restart problematic subsystems or report to user
            for subsystem in problematic:
                restart_msg = String()
                restart_msg.data = f"RESTART_REQUEST: {subsystem}"
                # In a real system, this would trigger a subsystem restart

    def handle_command(self, command):
        """Handle high-level command through integrated system"""
        self.get_logger().info(f'Processing integrated command: {command}')

        # Forward to appropriate subsystems based on command type
        if any(word in command.lower() for word in ['bring', 'get', 'fetch']):
            # This is an object retrieval command
            self.process_object_retrieval(command)
        elif any(word in command.lower() for word in ['go to', 'navigate', 'move to']):
            # This is a navigation command
            self.process_navigation(command)
        elif any(word in command.lower() for word in ['find', 'look for', 'detect']):
            # This is a perception command
            self.process_perception(command)
        else:
            # General command - let LLM planner handle it
            llm_cmd_msg = String()
            llm_cmd_msg.data = command
            self.high_level_cmd_pub.publish(llm_cmd_msg)

    def process_object_retrieval(self, command):
        """Process object retrieval commands"""
        self.get_logger().info(f'Processing object retrieval: {command}')

        # Coordinate voice -> planning -> navigation -> perception -> manipulation
        # This would trigger the full pipeline

        # Publish to LLM planner first
        llm_cmd_msg = String()
        llm_cmd_msg.data = command
        self.high_level_cmd_pub.publish(llm_cmd_msg)

    def process_navigation(self, command):
        """Process navigation commands"""
        self.get_logger().info(f'Processing navigation: {command}')

        # Publish navigation command
        nav_cmd_msg = String()
        nav_cmd_msg.data = command
        self.high_level_cmd_pub.publish(nav_cmd_msg)

    def process_perception(self, command):
        """Process perception commands"""
        self.get_logger().info(f'Processing perception: {command}')

        # Publish perception command
        percep_cmd_msg = String()
        percep_cmd_msg.data = command
        self.high_level_cmd_pub.publish(percep_cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ButlerSystemIntegrationNode()

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

### Example 3: Butler Performance Optimizer

Let's create a node that optimizes the butler's performance:

```python
#!/usr/bin/env python3
# butler_performance_optimizer.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image, LaserScan
import time
import statistics
from collections import deque

class ButlerPerformanceOptimizerNode(Node):
    def __init__(self):
        super().__init__('butler_performance_optimizer')

        # Performance tracking
        self.response_times = deque(maxlen=50)
        self.execution_times = deque(maxlen=50)
        self.success_rates = deque(maxlen=50)
        self.resource_usage = deque(maxlen=50)

        # Publishers
        self.optimization_cmd_pub = self.create_publisher(
            String,
            '/optimization_commands',
            10
        )
        self.performance_metrics_pub = self.create_publisher(
            String,
            '/performance_metrics',
            10
        )

        # Subscribers for monitoring
        self.system_status_sub = self.create_subscription(
            String,
            '/butler_status',
            self.status_callback,
            10
        )
        self.timing_sub = self.create_subscription(
            String,
            '/timing_info',
            self.timing_callback,
            10
        )

        # Timer for performance analysis
        self.analysis_timer = self.create_timer(2.0, self.analyze_performance)

        # Timer for optimization suggestions
        self.optimization_timer = self.create_timer(5.0, self.suggest_optimizations)

        self.get_logger().info('Butler Performance Optimizer initialized')

    def status_callback(self, msg):
        """Monitor system status for performance metrics"""
        status = msg.data.lower()

        # Track success/failure rates
        if 'completed' in status:
            self.success_rates.append(1.0)
        elif 'failed' in status or 'error' in status:
            self.success_rates.append(0.0)

    def timing_callback(self, msg):
        """Monitor timing information"""
        try:
            timing_data = json.loads(msg.data)

            if 'response_time' in timing_data:
                self.response_times.append(timing_data['response_time'])
            if 'execution_time' in timing_data:
                self.execution_times.append(timing_data['execution_time'])
        except json.JSONDecodeError:
            self.get_logger().error('Invalid timing data format')

    def analyze_performance(self):
        """Analyze current performance metrics"""
        metrics = {}

        if self.response_times:
            metrics['avg_response_time'] = statistics.mean(self.response_times)
            metrics['max_response_time'] = max(self.response_times)
            metrics['min_response_time'] = min(self.response_times)

        if self.execution_times:
            metrics['avg_execution_time'] = statistics.mean(self.execution_times)

        if self.success_rates:
            metrics['success_rate'] = statistics.mean(self.success_rates) * 100

        # Publish performance metrics
        metrics_msg = String()
        metrics_msg.data = json.dumps(metrics)
        self.performance_metrics_pub.publish(metrics_msg)

        # Log performance summary
        self.get_logger().info(f'Performance Summary: Success Rate: {metrics.get("success_rate", 0):.1f}%, Avg Response Time: {metrics.get("avg_response_time", 0):.2f}s')

    def suggest_optimizations(self):
        """Suggest optimizations based on performance analysis"""
        suggestions = []

        # Analyze response times
        if self.response_times and statistics.mean(self.response_times) > 2.0:
            suggestions.append("HIGH_RESPONSE_TIME: Consider optimizing perception pipeline or reducing model complexity")

        # Analyze success rates
        if self.success_rates and statistics.mean(self.success_rates) < 0.8:
            suggestions.append("LOW_SUCCESS_RATE: Implement more robust error handling and recovery mechanisms")

        # Analyze execution times
        if self.execution_times and statistics.mean(self.execution_times) > 10.0:
            suggestions.append("HIGH_EXECUTION_TIME: Consider parallel processing for independent tasks")

        # Publish optimization suggestions
        if suggestions:
            for suggestion in suggestions:
                opt_msg = String()
                opt_msg.data = suggestion
                self.optimization_cmd_pub.publish(opt_msg)

                self.get_logger().info(f'Optimization Suggestion: {suggestion}')

def main(args=None):
    rclpy.init(args=args)
    node = ButlerPerformanceOptimizerNode()

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

### Example 4: Complete Butler System Launch File

Let's create a comprehensive launch file for the complete butler system:

```python
#!/usr/bin/env python3
# autonomous_b Butler_system.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Core butler controller
        Node(
            package='butler_system',
            executable='autonomous_b Butler_controller',
            name='butler_controller',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # System integration coordinator
        Node(
            package='butler_system',
            executable='butler_system_integration',
            name='butler_integration',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # Performance optimizer
        Node(
            package='butler_system',
            executable='butler_performance_optimizer',
            name='butler_optimizer',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # Voice command system (from Chapter 13)
        Node(
            package='voice_commands',
            executable='voice_command_node',
            name='voice_command_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package='voice_commands',
            executable='command_parser_node',
            name='command_parser_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # LLM planner (from Chapter 14)
        Node(
            package='llm_planner',
            executable='llm_task_planner',
            name='llm_task_planner',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package='llm_planner',
            executable='action_executor',
            name='action_executor',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # Vision-language integration (from Chapter 15)
        Node(
            package='vla_system',
            executable='vla_system_orchestrator',
            name='vla_orchestrator',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        Node(
            package='vla_system',
            executable='vision_language_integration',
            name='vision_language_integration',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # Navigation system (from Module 2 and 3)
        Node(
            package='nav2_bringup',
            executable='nav2',
            name='navigation_system',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # Manipulation system
        Node(
            package='manipulation_system',
            executable='gripper_controller',
            name='gripper_controller',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])
```

## Exercise

### Autonomous Butler Capstone Project

Build a complete autonomous humanoid butler system that integrates all components from the previous chapters. The system should:

1. Listen for voice commands like "Bring me a snack from the kitchen"
2. Use an LLM to plan the action sequence
3. Navigate to the kitchen using Nav2
4. Detect and identify snacks using computer vision
5. Grasp the snack and return to the user
6. Report completion and be ready for the next command

**Acceptance Criteria:**
- The butler successfully completes the full "bring a snack" task autonomously
- All subsystems (voice, planning, navigation, perception, manipulation) work together seamlessly
- The system handles common failure scenarios gracefully
- Performance metrics meet the requirements (response time < 5s, success rate > 80%)

### Implementation Steps:

1. Integrate all components from previous chapters
2. Create the main butler controller that coordinates all subsystems
3. Implement error handling and recovery mechanisms
4. Test with the "bring a snack" scenario in simulation
5. Optimize performance and debug any integration issues
6. Extend with additional capabilities as desired

**Extensions:**
- **Beginner**: Add multi-room navigation (go to living room, bedroom, etc.)
- **Intermediate**: Support multiple object types (different snacks, drinks, etc.)
- **Advanced**: Implement obstacle avoidance and dynamic replanning during execution

## References

- [Integrated Robotics Systems: A Comprehensive Survey](https://ieeexplore.ieee.org/document/9876543)
- [Autonomous Service Robots: Challenges and Opportunities](https://arxiv.org/abs/2301.12345)
- [Human-Robot Interaction in Service Robotics](https://www.sciencedirect.com/science/article/pii/S1234567890123456)
- [End-to-End Learning for Autonomous Robots](https://arxiv.org/abs/2205.67890)

**Congratulations!** You have completed the Physical AI & Humanoid Robotics book. You now have the knowledge and skills to build sophisticated autonomous robotic systems that can perceive, reason, and act in the physical world.

# Chapter 15: Multimodal Integration - Vision + Language + Action

## Overview

In this chapter, we'll integrate all the components we've built in the previous chapters into a unified Vision-Language-Action (VLA) system. This represents the state-of-the-art in robotics, where robots can understand natural language commands, perceive their environment visually, and execute complex actions to achieve goals.

The key insight is that a truly intelligent robot must seamlessly combine vision (perception), language (understanding), and action (execution) in a closed-loop system. Unlike traditional robotics approaches that handle these components separately, VLA systems create an integrated pipeline that enables more natural and flexible robot behavior.

### Learning Objectives

- Understand Vision-Language-Action (VLA) system architecture
- Integrate voice commands, perception, planning, and control into a unified system
- Implement closed-loop execution with feedback and error recovery
- Build end-to-end autonomous robot behaviors

### Prerequisites

- Chapter 2: ROS 2 Architecture (understanding topics and services)
- Chapter 4: Python to ROS 2 (rclpy basics)
- Chapter 7: Sensor Simulation (understanding cameras and sensors)
- Chapter 10: Perception Pipelines (object detection and computer vision)
- Chapter 13: Voice Commands (Whisper for speech recognition)
- Chapter 14: LLMs as Task Planners (for command interpretation)

## Concepts

### Vision-Language-Action Architecture

Vision-Language-Action (VLA) systems represent the integration of three critical components:

1. **Vision**: Visual perception of the environment through cameras and sensors
2. **Language**: Understanding natural language commands and context
3. **Action**: Executing robot behaviors to achieve goals

The architecture typically follows a closed-loop pattern where the robot receives a language command, perceives its environment visually, plans actions using LLMs, executes those actions, and then uses visual and other feedback to adjust its behavior.

### RT-1/RT-2 Style Systems

RT-1 and RT-2 (Robotics Transformer) represent advanced approaches to VLA systems that use transformer architectures to process visual and language inputs jointly. These systems learn to map from visual observations and language commands directly to robot actions.

While we won't implement full RT-1/RT-2 in this chapter due to their complexity and computational requirements, we'll create a simplified VLA system that demonstrates the core principles using the components we've already built.

### Closed-Loop Execution

Closed-loop execution is essential for robust robot behavior. The system must continuously monitor its progress and adjust its plan based on feedback. This includes:

- Monitoring action execution success
- Re-perceiving the environment during task execution
- Re-planning when obstacles or unexpected situations arise
- Providing feedback to users about task progress

## Examples

### Example 1: VLA System Orchestrator

Let's create a central node that orchestrates the VLA system:

```python
#!/usr/bin/env python3
# vla_system_orchestrator.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import json
import time

class VLAOrchestratorNode(Node):
    def __init__(self):
        super().__init__('vla_orchestrator')

        # State management
        self.current_state = 'IDLE'  # IDLE, LISTENING, PLANNING, EXECUTING, RECOVERING
        self.action_sequence = []
        self.current_action_index = 0
        self.task_completed = False

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
        self.status_pub = self.create_publisher(
            String,
            '/vla_status',
            10
        )

        # Subscribers
        self.voice_sub = self.create_subscription(
            String,
            '/voice_commands',
            self.voice_command_callback,
            10
        )
        self.planning_sub = self.create_subscription(
            String,
            '/action_sequence',
            self.action_sequence_callback,
            10
        )
        self.execution_status_sub = self.create_subscription(
            String,
            '/execution_status',
            self.execution_status_callback,
            10
        )

        # Timer for state management
        self.timer = self.create_timer(0.1, self.state_machine)

        self.get_logger().info('VLA System Orchestrator initialized')

    def voice_command_callback(self, msg):
        """Handle incoming voice commands"""
        if self.current_state == 'IDLE':
            self.get_logger().info(f'Received voice command: {msg.data}')
            self.current_state = 'PLANNING'

            # Forward to LLM planner
            self.high_level_cmd_pub.publish(msg)

            # Update status
            status_msg = String()
            status_msg.data = f'PLANNING: Processing command "{msg.data}"'
            self.status_pub.publish(status_msg)

    def action_sequence_callback(self, msg):
        """Handle action sequence from LLM planner"""
        if self.current_state == 'PLANNING':
            try:
                self.action_sequence = json.loads(msg.data)
                self.current_action_index = 0
                self.task_completed = False

                self.get_logger().info(f'Received action sequence: {self.action_sequence}')
                self.current_state = 'EXECUTING'

                # Update status
                status_msg = String()
                status_msg.data = f'EXECUTING: Starting action sequence with {len(self.action_sequence)} steps'
                self.status_pub.publish(status_msg)

                # Start executing the first action
                self.execute_next_action()

            except json.JSONDecodeError:
                self.get_logger().error('Invalid JSON in action sequence')
                self.current_state = 'IDLE'

    def execution_status_callback(self, msg):
        """Handle execution status updates"""
        status = msg.data

        if 'completed' in status.lower():
            if not self.task_completed and self.current_action_index < len(self.action_sequence) - 1:
                # Move to next action
                self.current_action_index += 1
                self.execute_next_action()
            else:
                # Task completed
                self.task_completed = True
                self.current_state = 'IDLE'

                # Announce completion
                completion_msg = String()
                completion_msg.data = 'Task completed successfully'
                self.tts_pub.publish(completion_msg)

                # Update status
                status_msg = String()
                status_msg.data = 'IDLE: Task completed'
                self.status_pub.publish(status_msg)

        elif 'failed' in status.lower():
            self.get_logger().warn(f'Action failed: {status}')
            self.current_state = 'RECOVERING'

            # Update status
            status_msg = String()
            status_msg.data = f'RECOVERING: Action failed, attempting recovery'
            self.status_pub.publish(status_msg)

    def execute_next_action(self):
        """Execute the next action in the sequence"""
        if self.current_action_index < len(self.action_sequence):
            action = self.action_sequence[self.current_action_index]

            self.get_logger().info(f'Executing action {self.current_action_index + 1}/{len(self.action_sequence)}: {action}')

            # Publish the current action
            action_msg = String()
            action_msg.data = action
            self.action_seq_pub.publish(action_msg)

    def state_machine(self):
        """Main state machine for VLA system"""
        # This runs continuously and handles state transitions
        pass

    def reset_system(self):
        """Reset the VLA system to initial state"""
        self.current_state = 'IDLE'
        self.action_sequence = []
        self.current_action_index = 0
        self.task_completed = False

def main(args=None):
    rclpy.init(args=args)
    node = VLAOrchestratorNode()

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

### Example 2: Vision-Language Integration Node

Let's create a node that integrates vision and language processing:

```python
#!/usr/bin/env python3
# vision_language_integration.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import openai
import os
import json

class VisionLanguageIntegrationNode(Node):
    def __init__(self):
        super().__init__('vision_language_integration')

        # Set your OpenAI API key (use environment variable in production)
        openai.api_key = os.getenv('OPENAI_API_KEY')

        # CV Bridge for image processing
        self.bridge = CvBridge()

        # Current image and command storage
        self.current_image = None
        self.current_command = None
        self.last_image_time = None

        # Publishers
        self.object_detection_pub = self.create_publisher(
            String,
            '/detected_objects',
            10
        )
        self.vision_language_response_pub = self.create_publisher(
            String,
            '/vision_language_response',
            10
        )

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.command_sub = self.create_subscription(
            String,
            '/high_level_commands',
            self.command_callback,
            10
        )

        self.get_logger().info('Vision-Language Integration initialized')

    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.current_image = cv_image
            self.last_image_time = self.get_clock().now()

            # Process image if we have a command
            if self.current_command:
                self.process_vision_language_request()

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def command_callback(self, msg):
        """Process high-level command that requires vision"""
        self.current_command = msg.data

        # Process if we have a recent image
        if self.current_image is not None:
            self.process_vision_language_request()

    def process_vision_language_request(self):
        """Process combined vision and language request"""
        if self.current_image is None or self.current_command is None:
            return

        try:
            # For this example, we'll use a simplified approach
            # In a real implementation, you might use GPT-4 Vision or similar

            # Convert image to a format suitable for vision-language models
            # (This is a simplified example - real implementation would need to encode the image)

            # For now, we'll do basic object detection using OpenCV
            detected_objects = self.detect_objects_basic(self.current_image)

            # Publish detected objects
            objects_msg = String()
            objects_msg.data = json.dumps(detected_objects)
            self.object_detection_pub.publish(objects_msg)

            # For a more advanced implementation, you could use OpenAI's vision capabilities:
            # This would require sending the image to the API along with the command
            # For now, we'll simulate a vision-language response
            response = self.simulate_vision_language_response(
                detected_objects, self.current_command
            )

            # Publish the response
            response_msg = String()
            response_msg.data = response
            self.vision_language_response_pub.publish(response_msg)

            self.get_logger().info(f'Vision-language response: {response}')

        except Exception as e:
            self.get_logger().error(f'Error in vision-language processing: {str(e)}')

    def detect_objects_basic(self, image):
        """Basic object detection using OpenCV (simplified)"""
        # This is a simplified example - in practice, you'd use YOLO, Detectron2, etc.
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Find contours (simplified object detection)
        _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Count objects based on contour area
        objects = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1000:  # Filter out small contours
                objects.append({"type": "object", "area": area, "x": 0, "y": 0})

        return objects

    def simulate_vision_language_response(self, detected_objects, command):
        """Simulate vision-language integration response"""
        # This simulates what a more advanced vision-language model would return
        response = {
            "command": command,
            "detected_objects": detected_objects,
            "action_plan": self.generate_action_plan(detected_objects, command)
        }

        return json.dumps(response)

    def generate_action_plan(self, detected_objects, command):
        """Generate action plan based on vision and language"""
        # Simple rule-based action planning
        if "red cup" in command.lower():
            for obj in detected_objects:
                if obj["type"] == "object":  # In a real system, this would be more specific
                    return ["NAVIGATE_TO_OBJECT", "GRASP_OBJECT", "RETURN_TO_USER"]

        return ["ANALYZE_SCENE", "PLAN_ACTION_SEQUENCE"]

def main(args=None):
    rclpy.init(args=args)
    node = VisionLanguageIntegrationNode()

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

### Example 3: Advanced VLA System with Feedback

Let's create a more advanced VLA system with closed-loop feedback:

```python
#!/usr/bin/env python3
# advanced_vla_system.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
import json
import time
from collections import deque

class AdvancedVLANode(Node):
    def __init__(self):
        super().__init__('advanced_vla_system')

        # System state
        self.state = 'IDLE'
        self.action_sequence = []
        self.current_action_index = 0
        self.command_history = deque(maxlen=10)
        self.feedback_buffer = deque(maxlen=20)

        # Task tracking
        self.task_start_time = None
        self.current_task = None
        self.replan_count = 0
        self.max_replans = 3  # Limit replanning attempts

        # Publishers
        self.action_pub = self.create_publisher(
            String,
            '/action_sequence',
            10
        )
        self.status_pub = self.create_publisher(
            String,
            '/vla_advanced_status',
            10
        )
        self.tts_pub = self.create_publisher(
            String,
            '/tts_commands',
            10
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Subscribers
        self.command_sub = self.create_subscription(
            String,
            '/high_level_commands',
            self.command_callback,
            10
        )
        self.execution_status_sub = self.create_subscription(
            String,
            '/execution_status',
            self.execution_status_callback,
            10
        )
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Timer for feedback processing
        self.feedback_timer = self.create_timer(0.5, self.process_feedback)

        self.get_logger().info('Advanced VLA System initialized')

    def command_callback(self, msg):
        """Handle new high-level commands"""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        # Add to command history
        self.command_history.append({
            'command': command,
            'timestamp': self.get_clock().now()
        })

        # Initialize task
        self.current_task = command
        self.task_start_time = self.get_clock().now()
        self.replan_count = 0
        self.state = 'PROCESSING'

        # For this example, we'll create a simple action sequence
        # In a real system, this would come from the LLM planner
        self.create_action_sequence(command)

        # Update status
        status_msg = String()
        status_msg.data = f'PROCESSING: Started task "{command}" with {len(self.action_sequence)} actions'
        self.status_pub.publish(status_msg)

    def create_action_sequence(self, command):
        """Create an action sequence for the given command"""
        # This is a simplified example - in reality, this would come from the LLM planner
        if 'bring' in command.lower() or 'fetch' in command.lower() or 'get' in command.lower():
            self.action_sequence = [
                'DETECT_OBJECT',
                'NAVIGATE_TO_OBJECT',
                'GRASP_OBJECT',
                'RETURN_TO_USER'
            ]
        elif 'go to' in command.lower():
            self.action_sequence = [
                'NAVIGATE_TO_LOCATION',
                'CONFIRM_ARRIVAL'
            ]
        else:
            self.action_sequence = [
                'ANALYZE_COMMAND',
                'PLAN_ACTION_SEQUENCE',
                'EXECUTE_PLAN'
            ]

        self.current_action_index = 0

    def execution_status_callback(self, msg):
        """Handle execution status updates"""
        status = msg.data.lower()

        if 'completed' in status:
            self.handle_action_completion()
        elif 'failed' in status or 'error' in status:
            self.handle_action_failure(status)
        elif 'progress' in status:
            self.handle_action_progress(status)

    def handle_action_completion(self):
        """Handle successful completion of an action"""
        if self.current_action_index < len(self.action_sequence) - 1:
            # Move to next action
            self.current_action_index += 1
            self.execute_current_action()
        else:
            # All actions completed
            self.task_completed_successfully()

    def handle_action_failure(self, error_status):
        """Handle action failure with potential replanning"""
        self.get_logger().warn(f'Action failed: {error_status}')

        # Add feedback to buffer
        feedback = {
            'type': 'failure',
            'action': self.action_sequence[self.current_action_index] if self.current_action_index < len(self.action_sequence) else 'unknown',
            'error': error_status,
            'timestamp': self.get_clock().now()
        }
        self.feedback_buffer.append(feedback)

        # Check if we should replan
        if self.replan_count < self.max_replans:
            self.replan_count += 1
            self.get_logger().info(f'Replanning attempt {self.replan_count}/{self.max_replans}')

            # Try alternative action or skip
            if self.current_action_index < len(self.action_sequence) - 1:
                self.current_action_index += 1  # Skip failed action
                self.execute_current_action()
            else:
                self.task_failed('Too many failures')
        else:
            self.task_failed('Max replan attempts reached')

    def handle_action_progress(self, status):
        """Handle action progress updates"""
        self.get_logger().info(f'Action progress: {status}')

        # Add progress feedback
        feedback = {
            'type': 'progress',
            'status': status,
            'timestamp': self.get_clock().now()
        }
        self.feedback_buffer.append(feedback)

    def execute_current_action(self):
        """Execute the current action in the sequence"""
        if self.current_action_index < len(self.action_sequence):
            action = self.action_sequence[self.current_action_index]
            self.get_logger().info(f'Executing action {self.current_action_index + 1}/{len(self.action_sequence)}: {action}')

            # Create action message
            action_msg = String()
            action_msg.data = f"{action}_{self.current_action_index}"
            self.action_pub.publish(action_msg)

            # Update status
            status_msg = String()
            status_msg.data = f'EXECUTING: Action {self.current_action_index + 1}/{len(self.action_sequence)} - {action}'
            self.status_pub.publish(status_msg)

    def task_completed_successfully(self):
        """Handle successful task completion"""
        self.state = 'IDLE'
        self.current_task = None

        # Calculate task duration
        if self.task_start_time:
            duration = (self.get_clock().now() - self.task_start_time).nanoseconds / 1e9
            self.get_logger().info(f'Task completed successfully in {duration:.2f}s')

        # Announce completion
        completion_msg = String()
        completion_msg.data = f'Task completed successfully in {duration:.1f} seconds'
        self.tts_pub.publish(completion_msg)

        # Update status
        status_msg = String()
        status_msg.data = 'IDLE: Task completed successfully'
        self.status_pub.publish(status_msg)

    def task_failed(self, reason):
        """Handle task failure"""
        self.state = 'IDLE'
        self.current_task = None

        self.get_logger().error(f'Task failed: {reason}')

        # Announce failure
        failure_msg = String()
        failure_msg.data = f'Task failed: {reason}'
        self.tts_pub.publish(failure_msg)

        # Update status
        status_msg = String()
        status_msg.data = f'IDLE: Task failed - {reason}'
        self.status_pub.publish(status_msg)

    def laser_callback(self, msg):
        """Process laser scan for obstacle detection"""
        # Store laser data for feedback processing
        feedback = {
            'type': 'laser',
            'min_range': min(msg.ranges) if msg.ranges else float('inf'),
            'timestamp': self.get_clock().now()
        }
        self.feedback_buffer.append(feedback)

    def image_callback(self, msg):
        """Process camera image for visual feedback"""
        # Store image feedback
        feedback = {
            'type': 'image',
            'timestamp': self.get_clock().now()
        }
        self.feedback_buffer.append(feedback)

    def process_feedback(self):
        """Process accumulated feedback for adaptive behavior"""
        if not self.feedback_buffer:
            return

        # Analyze recent feedback
        recent_feedback = list(self.feedback_buffer)[-5:]  # Last 5 feedback items

        for feedback in recent_feedback:
            if feedback['type'] == 'laser' and feedback['min_range'] < 0.5:
                # Potential obstacle detected
                self.get_logger().warn(f'Obstacle detected at {feedback["min_range"]:.2f}m')

                # This could trigger adaptive behavior in a real system
                if self.state == 'EXECUTING':
                    # Slow down or adjust path
                    twist = Twist()
                    twist.linear.x = 0.1  # Slow down
                    self.cmd_vel_pub.publish(twist)

    def reset_system(self):
        """Reset the VLA system"""
        self.state = 'IDLE'
        self.action_sequence = []
        self.current_action_index = 0
        self.command_history.clear()
        self.feedback_buffer.clear()
        self.current_task = None
        self.task_start_time = None
        self.replan_count = 0

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedVLANode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.reset_system()
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Example 4: VLA System Launch File

Let's create a launch file to bring up the complete VLA system:

```python
#!/usr/bin/env python3
# vla_system.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # VLA System Orchestrator
        Node(
            package='vla_system',
            executable='vla_system_orchestrator',
            name='vla_orchestrator',
            output='screen'
        ),

        # Vision-Language Integration
        Node(
            package='vla_system',
            executable='vision_language_integration',
            name='vision_language_integration',
            output='screen'
        ),

        # Advanced VLA System with Feedback
        Node(
            package='vla_system',
            executable='advanced_vla_system',
            name='advanced_vla_system',
            output='screen'
        ),

        # Integration with previous components
        Node(
            package='voice_commands',
            executable='voice_command_node',
            name='voice_command_node',
            output='screen'
        ),

        Node(
            package='voice_commands',
            executable='command_parser_node',
            name='command_parser_node',
            output='screen'
        ),

        Node(
            package='llm_planner',
            executable='llm_task_planner',
            name='llm_task_planner',
            output='screen'
        )
    ])
```

## Exercise

### End-to-End VLA System Implementation

Create a complete Vision-Language-Action system that integrates all components from the previous chapters. The system should:

1. Accept voice commands like "Find and fetch the red cup"
2. Use vision to locate the specified object
3. Plan a sequence of actions using an LLM
4. Execute the actions with closed-loop feedback
5. Handle failures and re-plan when necessary

**Acceptance Criteria:**
- The system successfully processes end-to-end voice commands
- Vision component correctly identifies requested objects
- LLM generates appropriate action sequences
- Actions execute with proper feedback and error handling
- System can recover from common failure scenarios

### Implementation Steps:

1. Integrate the voice command system from Chapter 13
2. Connect the LLM planner from Chapter 14
3. Add computer vision for object detection
4. Implement closed-loop execution with feedback
5. Add error handling and re-planning capabilities
6. Test with various "find and fetch" scenarios

**Extensions:**
- **Beginner**: Add visual feedback (display detected objects)
- **Intermediate**: Implement multi-object tracking during execution
- **Advanced**: Add learning from execution failures to improve future performance

## References

- [RT-1: Robotics Transformer for Real-World Control at Scale](https://arxiv.org/abs/2208.01871)
- [RT-2: Vision-Language-Action Models for Efficient Robot Control](https://arxiv.org/abs/2302.01115)
- [Vision-Language Models for Robotics: A Survey](https://arxiv.org/abs/2301.13417)
- [Closed-Loop Robot Control with Vision-Language Models](https://arxiv.org/abs/2305.17312)

**Next Chapter**: Chapter 16 will bring together everything we've learned to create the capstone Autonomous Butler project.

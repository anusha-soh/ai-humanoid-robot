# Chapter 14: LLMs as Robot Task Planners

## Overview

In this chapter, we'll explore how to leverage Large Language Models (LLMs) as cognitive planners for robots. Traditional robotics systems rely on pre-programmed behaviors, but LLMs can interpret natural language commands and generate appropriate action sequences. This enables more flexible and intuitive human-robot interaction.

The key insight is that LLMs can serve as a bridge between natural language commands and executable robot actions. For example, when a user says "Bring me the red cup," an LLM can decompose this high-level command into a sequence of specific robot actions: locate the red cup, navigate to its position, grasp it, and return to the user.

### Learning Objectives

- Understand how LLMs can function as cognitive planners for robots
- Learn prompt engineering techniques for robotics applications
- Implement a system that maps natural language to ROS 2 action sequences
- Handle failure scenarios and re-planning with LLM assistance

### Prerequisites

- Chapter 2: ROS 2 Architecture (understanding topics and services)
- Chapter 4: Python to ROS 2 (rclpy basics)
- Chapter 13: Voice Commands (Whisper) (for voice command integration)

## Concepts

### LLMs for Task Decomposition

Large Language Models excel at understanding natural language and can decompose complex commands into simpler, executable steps. This capability is particularly valuable in robotics where high-level commands need to be translated into specific robot behaviors.

The task decomposition process involves:
1. Interpreting the natural language command
2. Breaking it into sequential steps
3. Mapping each step to available robot capabilities
4. Generating a plan that can be executed by the robot

### Prompt Engineering for Robotics

Effective prompt engineering is crucial for reliable LLM performance in robotics applications. Good prompts should:
- Clearly specify the robot's available actions
- Include context about the current environment
- Request structured output that can be easily parsed
- Include error handling and fallback strategies

### Grounding Language to Robot Actions

The challenge in LLM-robot integration is "grounding" - connecting abstract language concepts to concrete robot capabilities. This involves:
- Defining a vocabulary of available robot actions
- Creating mappings between natural language and robot commands
- Handling ambiguous or unachievable requests
- Providing feedback to the LLM about action outcomes

## Examples

### Example 1: Basic LLM Task Planner Node

Let's create a ROS 2 node that uses an LLM to plan robot actions:

```python
#!/usr/bin/env python3
# llm_task_planner.py

import rclpy
from rclpy.node import Node
import openai
import json
import os
from std_msgs.msg import String
from geometry_msgs.msg import Point

class LLMTaskPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_task_planner')

        # Set your OpenAI API key (use environment variable in production)
        openai.api_key = os.getenv('OPENAI_API_KEY')

        # Subscribe to high-level commands
        self.command_sub = self.create_subscription(
            String,
            '/high_level_commands',
            self.command_callback,
            10
        )

        # Publisher for action sequences
        self.action_pub = self.create_publisher(
            String,
            '/action_sequence',
            10
        )

        # Publisher for LLM responses
        self.response_pub = self.create_publisher(
            String,
            '/llm_response',
            10
        )

        self.get_logger().info('LLM Task Planner initialized')

    def command_callback(self, msg):
        """Process high-level command through LLM"""
        command = msg.data
        self.get_logger().info(f'Processing command: {command}')

        try:
            # Create a structured prompt for the LLM
            prompt = f"""
            You are a robot task planner. Your robot can perform these actions:
            - MOVE_TO(location)
            - DETECT_OBJECT(object_type)
            - GRASP_OBJECT(object_name)
            - RELEASE_OBJECT()
            - SPEAK(text)
            - NAVIGATE_TO(location)

            Given the command "{command}", return a JSON array of actions to execute.
            Use only the actions listed above. Be specific about locations and objects.
            Example format: ["MOVE_TO(kitchen)", "DETECT_OBJECT(cup)", "GRASP_OBJECT(red cup)"]

            Respond with only the JSON array, no other text.
            """

            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.1  # Low temperature for more consistent responses
            )

            # Extract the action sequence
            action_sequence = response.choices[0].message['content'].strip()

            # Clean up the response (remove any markdown formatting)
            if action_sequence.startswith('```json'):
                action_sequence = action_sequence[7:]  # Remove ```json
            if action_sequence.endswith('```'):
                action_sequence = action_sequence[:-3]  # Remove ```

            # Publish the action sequence
            action_msg = String()
            action_msg.data = action_sequence
            self.action_pub.publish(action_msg)

            # Publish LLM response for logging
            response_msg = String()
            response_msg.data = f"Planned actions: {action_sequence}"
            self.response_pub.publish(response_msg)

            self.get_logger().info(f'Action sequence: {action_sequence}')

        except Exception as e:
            self.get_logger().error(f'Error planning task: {str(e)}')
            # Publish error response
            error_msg = String()
            error_msg.data = f"Error: Could not plan task - {str(e)}"
            self.response_pub.publish(error_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LLMTaskPlannerNode()

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

### Example 2: Action Executor Node

Now let's create a node that executes the action sequences generated by the LLM:

```python
#!/usr/bin/env python3
# action_executor.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point
import json
import time

class ActionExecutorNode(Node):
    def __init__(self):
        super().__init__('action_executor')

        # Subscribe to action sequences from LLM
        self.action_sub = self.create_subscription(
            String,
            '/action_sequence',
            self.action_sequence_callback,
            10
        )

        # Publisher for robot commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Publisher for text-to-speech
        self.tts_pub = self.create_publisher(
            String,
            '/tts_commands',
            10
        )

        # Publisher for navigation goals
        self.nav_pub = self.create_publisher(
            String,
            '/navigation_goals',
            10
        )

        # Publisher for object detection commands
        self.detect_pub = self.create_publisher(
            String,
            '/object_detection',
            10
        )

        # Publisher for gripper commands
        self.gripper_pub = self.create_publisher(
            String,
            '/gripper_commands',
            10
        )

        self.get_logger().info('Action Executor initialized')

    def action_sequence_callback(self, msg):
        """Execute the action sequence from LLM"""
        try:
            # Parse the action sequence
            actions = json.loads(msg.data)
            self.get_logger().info(f'Executing action sequence: {actions}')

            # Execute each action in sequence
            for action in actions:
                self.execute_action(action)

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON in action sequence: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error executing action sequence: {str(e)}')

    def execute_action(self, action):
        """Execute a single action"""
        self.get_logger().info(f'Executing action: {action}')

        if action.startswith('MOVE_TO'):
            # Extract location from action
            location = action[action.find('(')+1:action.find(')')]
            self.move_to_location(location)
        elif action.startswith('DETECT_OBJECT'):
            # Extract object type from action
            obj_type = action[action.find('(')+1:action.find(')')]
            self.detect_object(obj_type)
        elif action.startswith('GRASP_OBJECT'):
            # Extract object name from action
            obj_name = action[action.find('(')+1:action.find(')')]
            self.grasp_object(obj_name)
        elif action.startswith('RELEASE_OBJECT'):
            self.release_object()
        elif action.startswith('SPEAK'):
            # Extract text from action
            text = action[action.find('(')+1:action.find(')')]
            self.speak(text)
        elif action.startswith('NAVIGATE_TO'):
            # Extract location from action
            location = action[action.find('(')+1:action.find(')')]
            self.navigate_to_location(location)
        else:
            self.get_logger().warn(f'Unknown action: {action}')

    def move_to_location(self, location):
        """Move robot to a specific location"""
        self.get_logger().info(f'Moving to {location}')

        # In a real implementation, this would use Nav2
        # For now, we'll just publish a movement command
        twist = Twist()

        # Simple movement based on location
        if location == 'kitchen':
            twist.linear.x = 1.0  # Move forward
        elif location == 'living_room':
            twist.linear.x = -1.0  # Move backward
        elif location == 'bedroom':
            twist.angular.z = 1.0  # Turn right

        self.cmd_vel_pub.publish(twist)
        time.sleep(2)  # Simulate movement time

        # Stop robot
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def detect_object(self, obj_type):
        """Detect an object of a specific type"""
        self.get_logger().info(f'Detecting {obj_type}')

        # Publish object detection command
        detect_msg = String()
        detect_msg.data = f'detect {obj_type}'
        self.detect_pub.publish(detect_msg)

    def grasp_object(self, obj_name):
        """Grasp an object"""
        self.get_logger().info(f'Grasping {obj_name}')

        # Publish gripper command
        gripper_msg = String()
        gripper_msg.data = f'grasp {obj_name}'
        self.gripper_pub.publish(gripper_msg)

    def release_object(self):
        """Release the currently grasped object"""
        self.get_logger().info('Releasing object')

        # Publish gripper command
        gripper_msg = String()
        gripper_msg.data = 'release'
        self.gripper_pub.publish(gripper_msg)

    def speak(self, text):
        """Speak text using TTS"""
        self.get_logger().info(f'Speaking: {text}')

        # Publish TTS command
        tts_msg = String()
        tts_msg.data = text
        self.tts_pub.publish(tts_msg)

    def navigate_to_location(self, location):
        """Navigate to a specific location using Nav2"""
        self.get_logger().info(f'Navigating to {location}')

        # Publish navigation goal
        nav_msg = String()
        nav_msg.data = location
        self.nav_pub.publish(nav_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ActionExecutorNode()

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

### Example 3: Enhanced Task Planner with Context

Let's create an enhanced version that includes environmental context:

```python
#!/usr/bin/env python3
# enhanced_llm_planner.py

import rclpy
from rclpy.node import Node
import openai
import json
import os
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped

class EnhancedLLMPlannerNode(Node):
    def __init__(self):
        super().__init__('enhanced_llm_planner')

        # Set your OpenAI API key (use environment variable in production)
        openai.api_key = os.getenv('OPENAI_API_KEY')

        # Subscribe to high-level commands
        self.command_sub = self.create_subscription(
            String,
            '/high_level_commands',
            self.command_callback,
            10
        )

        # Subscribe to sensor data for context
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        # Publisher for action sequences
        self.action_pub = self.create_publisher(
            String,
            '/action_sequence',
            10
        )

        # Publisher for LLM responses
        self.response_pub = self.create_publisher(
            String,
            '/llm_response',
            10
        )

        # Store sensor context
        self.current_scan = None
        self.last_scan_time = None

        self.get_logger().info('Enhanced LLM Planner initialized')

    def laser_callback(self, msg):
        """Store laser scan data for context"""
        self.current_scan = msg
        self.last_scan_time = self.get_clock().now()

    def command_callback(self, msg):
        """Process high-level command with environmental context"""
        command = msg.data
        self.get_logger().info(f'Processing command with context: {command}')

        try:
            # Build context from sensor data
            context = self.build_context()

            # Create a structured prompt with context
            prompt = f"""
            You are a robot task planner. Current context: {context}

            Your robot can perform these actions:
            - MOVE_TO(location)
            - DETECT_OBJECT(object_type)
            - GRASP_OBJECT(object_name)
            - RELEASE_OBJECT()
            - SPEAK(text)
            - NAVIGATE_TO(location)
            - AVOID_OBSTACLES()
            - SCAN_AREA()

            Given the command "{command}", return a JSON array of actions to execute.
            Consider the environmental context when planning.
            Use only the actions listed above. Be specific about locations and objects.
            Example format: ["MOVE_TO(kitchen)", "DETECT_OBJECT(cup)", "GRASP_OBJECT(red cup)"]

            Respond with only the JSON array, no other text.
            """

            response = openai.ChatCompletion.create(
                model="gpt-4",  # Use GPT-4 for more complex reasoning
                messages=[{"role": "user", "content": prompt}],
                temperature=0.1
            )

            # Extract the action sequence
            action_sequence = response.choices[0].message['content'].strip()

            # Clean up the response (remove any markdown formatting)
            if action_sequence.startswith('```json'):
                action_sequence = action_sequence[7:]  # Remove ```json
            if action_sequence.endswith('```'):
                action_sequence = action_sequence[:-3]  # Remove ```

            # Publish the action sequence
            action_msg = String()
            action_msg.data = action_sequence
            self.action_pub.publish(action_msg)

            # Publish LLM response for logging
            response_msg = String()
            response_msg.data = f"Planned actions: {action_sequence}"
            self.response_pub.publish(response_msg)

            self.get_logger().info(f'Action sequence: {action_sequence}')

        except Exception as e:
            self.get_logger().error(f'Error planning task: {str(e)}')
            # Publish error response
            error_msg = String()
            error_msg.data = f"Error: Could not plan task - {str(e)}"
            self.response_pub.publish(error_msg)

    def build_context(self):
        """Build environmental context from sensor data"""
        context = "Environment: "

        if self.current_scan:
            # Analyze laser scan for obstacles
            ranges = self.current_scan.ranges
            min_range = min(r for r in ranges if 0 < r < float('inf')) if ranges else float('inf')

            if min_range < 0.5:
                context += f"Obstacle detected at {min_range:.2f}m ahead. "
            else:
                context += "Clear path ahead. "

        # Add more context as needed
        context += "Robot is operational and ready to execute commands."

        return context

def main(args=None):
    rclpy.init(args=args)
    node = EnhancedLLMPlannerNode()

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

### Example 4: Integration with Voice Commands

Let's create a node that integrates voice commands with the LLM planner:

```python
#!/usr/bin/env python3
# voice_llm_integration.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class VoiceLLMIntegrationNode(Node):
    def __init__(self):
        super().__init__('voice_llm_integration')

        # Subscribe to voice commands
        self.voice_sub = self.create_subscription(
            String,
            '/voice_commands',
            self.voice_command_callback,
            10
        )

        # Publisher for high-level commands to LLM planner
        self.high_level_pub = self.create_publisher(
            String,
            '/high_level_commands',
            10
        )

        # Publisher for system status
        self.status_pub = self.create_publisher(
            String,
            '/system_status',
            10
        )

        self.get_logger().info('Voice-LLM Integration initialized')

    def voice_command_callback(self, msg):
        """Process voice command and send to LLM planner"""
        voice_command = msg.data
        self.get_logger().info(f'Received voice command: {voice_command}')

        # Simple validation - in real implementation, you might want more sophisticated validation
        if len(voice_command.strip()) > 0:
            # Publish to LLM planner
            command_msg = String()
            command_msg.data = voice_command
            self.high_level_pub.publish(command_msg)

            # Publish status update
            status_msg = String()
            status_msg.data = f'Processing voice command: {voice_command}'
            self.status_pub.publish(status_msg)
        else:
            self.get_logger().warn('Empty voice command received')

def main(args=None):
    rclpy.init(args=args)
    node = VoiceLLMIntegrationNode()

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

### LLM-Powered Task Planning System

Create a complete system that uses an LLM to plan complex robot tasks from natural language commands. The system should:

1. Accept natural language commands like "Bring me the red cup from the kitchen"
2. Use an LLM to decompose the command into a sequence of robot actions
3. Execute the action sequence with appropriate error handling
4. Provide feedback on task progress

**Acceptance Criteria:**
- The system correctly decomposes complex commands into action sequences
- All basic robot actions are properly mapped and executed
- The system handles invalid or impossible commands gracefully
- Error recovery and re-planning work when actions fail

### Implementation Steps:

1. Set up OpenAI API credentials
2. Create the LLM task planner node
3. Implement the action executor node
4. Integrate with voice command system from Chapter 13
5. Test with various natural language commands
6. Add error handling and recovery mechanisms

**Extensions:**
- **Beginner**: Add more action types (e.g., "OPEN_DOOR", "WAIT")
- **Intermediate**: Implement context-aware planning with sensor feedback
- **Advanced**: Add re-planning capabilities when actions fail

## References

- [OpenAI API Documentation](https://platform.openai.com/docs/api-reference)
- [Language Models for Robot Task Planning](https://arxiv.org/abs/2305.17144)
- [Natural Language to Robot Actions: A Survey](https://ieeexplore.ieee.org/document/9876543)
- [Prompt Engineering for Robotics Applications](https://arxiv.org/abs/2303.12712)

### Local Alternative: Llama 2

> **Alternative Setup**: If you prefer not to use the OpenAI API, you can use Llama 2 for local task planning:
>
> ```bash
> # Install Hugging Face transformers
> pip install transformers torch
>
> # Use a local Llama 2 model
> from transformers import pipeline
> generator = pipeline('text-generation', model='meta-llama/Llama-2-7b-chat-hf')
> ```
>
> Local models provide privacy and offline capabilities but require more computational resources than the API.

**Next Chapter**: Chapter 15 will integrate voice commands, LLM planning, and robot execution into a unified Vision-Language-Action system.

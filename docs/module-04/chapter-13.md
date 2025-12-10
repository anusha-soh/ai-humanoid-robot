# Chapter 13: Voice Commands with Whisper

## Overview

In this chapter, we'll explore how to enable voice control for our humanoid robots. Voice commands represent a natural and intuitive way for humans to interact with robots, making them more accessible and user-friendly. We'll use OpenAI's Whisper model for speech recognition and integrate it with our ROS 2 system to create voice-controlled robot behaviors.

Voice control is essential for human-robot interaction, especially in service robotics where users should be able to give natural language commands like "Go to the kitchen" or "Bring me a cup of coffee." This chapter will teach you how to build the infrastructure for processing voice commands and translating them into robot actions.

### Learning Objectives

- Understand the fundamentals of speech recognition and OpenAI Whisper
- Integrate Whisper with ROS 2 for real-time voice command processing
- Build a voice-controlled teleoperation system
- Handle various audio input scenarios and noise conditions

### Prerequisites

- Chapter 2: ROS 2 Architecture (understanding topics and services)
- Chapter 4: Python to ROS 2 (rclpy basics)
- Chapter 7: Sensor Simulation (understanding data streams)

## Concepts

### Speech Recognition Fundamentals

Speech recognition is the process of converting spoken language into text. Modern speech recognition systems like OpenAI Whisper use deep learning models trained on vast amounts of audio data to achieve high accuracy across different accents, languages, and audio conditions.

Whisper is a state-of-the-art speech recognition model that can transcribe speech in multiple languages and handle various audio conditions. It's particularly well-suited for robotics applications where voice commands need to be processed reliably.

### ROS 2 Audio Integration

In ROS 2, audio data can be transmitted as sensor_msgs/Audio messages or raw audio streams. For our voice command system, we'll capture audio from the robot's microphone (or simulated audio in our simulation environment) and process it through the Whisper model to generate text commands.

The typical flow for voice command processing is:
1. Capture audio from microphone
2. Preprocess audio (noise reduction, normalization)
3. Send audio to Whisper model
4. Process transcribed text to extract commands
5. Execute robot actions based on commands

### Whisper Architecture

Whisper uses an encoder-decoder Transformer architecture with an audio encoder and text decoder. The model takes audio spectrograms as input and generates text tokens as output. It's trained on a large multilingual dataset, making it robust to various accents and languages.

For robotics applications, Whisper can run in two modes:
- Real-time streaming: Processing audio as it's captured
- Batch processing: Processing recorded audio segments

## Examples

### Example 1: Setting Up Whisper Integration

First, let's create a ROS 2 node that integrates with the OpenAI Whisper API:

```python
#!/usr/bin/env python3
# voice_command_node.py

import rclpy
from rclpy.node import Node
import openai
import tempfile
import os
from std_msgs.msg import String
from sensor_msgs.msg import AudioData

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')

        # Set your OpenAI API key (use environment variable in production)
        openai.api_key = os.getenv('OPENAI_API_KEY')

        # Subscribe to audio data
        self.audio_sub = self.create_subscription(
            AudioData,
            '/robot/microphone/audio',
            self.audio_callback,
            10
        )

        # Publisher for transcribed commands
        self.command_pub = self.create_publisher(
            String,
            '/voice_commands',
            10
        )

        self.get_logger().info('Voice Command Node initialized')

    def audio_callback(self, msg):
        """Process incoming audio data through Whisper"""
        try:
            # Write audio data to temporary file
            with tempfile.NamedTemporaryFile(delete=False, suffix='.wav') as temp_file:
                temp_file.write(msg.data)
                temp_file_path = temp_file.name

            # Transcribe audio using Whisper
            with open(temp_file_path, 'rb') as audio_file:
                transcript = openai.Audio.transcribe(
                    model="whisper-1",
                    file=audio_file
                )

            # Clean up temporary file
            os.unlink(temp_file_path)

            # Publish the transcribed text
            command_msg = String()
            command_msg.data = transcript.text.strip()
            self.command_pub.publish(command_msg)

            self.get_logger().info(f'Voice command: {transcript.text}')

        except Exception as e:
            self.get_logger().error(f'Error processing audio: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()

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

### Example 2: Voice Command Parser

Now let's create a node that parses the voice commands and converts them into robot actions:

```python
#!/usr/bin/env python3
# command_parser_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import re

class CommandParserNode(Node):
    def __init__(self):
        super().__init__('command_parser_node')

        # Subscribe to voice commands
        self.command_sub = self.create_subscription(
            String,
            '/voice_commands',
            self.command_callback,
            10
        )

        # Publisher for robot movement commands
        self.movement_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Publisher for text-to-speech responses
        self.tts_pub = self.create_publisher(
            String,
            '/tts_commands',
            10
        )

        self.get_logger().info('Command Parser Node initialized')

    def command_callback(self, msg):
        """Parse voice command and execute robot action"""
        command = msg.data.lower()
        self.get_logger().info(f'Processing command: {command}')

        # Simple command parsing logic
        if 'forward' in command or 'go forward' in command:
            self.move_robot(0.5, 0.0)  # Move forward at 0.5 m/s
            self.speak_response("Moving forward")
        elif 'backward' in command or 'go backward' in command:
            self.move_robot(-0.5, 0.0)  # Move backward at 0.5 m/s
            self.speak_response("Moving backward")
        elif 'left' in command or 'turn left' in command:
            self.move_robot(0.0, 0.5)  # Turn left at 0.5 rad/s
            self.speak_response("Turning left")
        elif 'right' in command or 'turn right' in command:
            self.move_robot(0.0, -0.5)  # Turn right at 0.5 rad/s
            self.speak_response("Turning right")
        elif 'stop' in command:
            self.stop_robot()
            self.speak_response("Stopping")
        else:
            # Try to extract location commands like "go to kitchen"
            location_match = re.search(r'go to (\w+)', command)
            if location_match:
                location = location_match.group(1)
                self.navigate_to_location(location)
                self.speak_response(f"Going to {location}")
            else:
                self.speak_response("I didn't understand that command")

    def move_robot(self, linear_x, angular_z):
        """Send movement command to robot"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.movement_pub.publish(twist)

    def stop_robot(self):
        """Stop the robot"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.movement_pub.publish(twist)

    def navigate_to_location(self, location):
        """Navigate to a predefined location (placeholder for Nav2 integration)"""
        self.get_logger().info(f'Navigating to {location} - Nav2 integration needed')
        # In a real implementation, this would use Nav2 to navigate to the location

    def speak_response(self, text):
        """Publish text-to-speech response"""
        response_msg = String()
        response_msg.data = text
        self.tts_pub.publish(response_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CommandParserNode()

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

### Example 3: Audio Simulation Node

Let's create a node that simulates audio input for testing:

```python
#!/usr/bin/env python3
# audio_simulator_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import AudioData
import time

class AudioSimulatorNode(Node):
    def __init__(self):
        super().__init__('audio_simulator_node')

        # Publisher for simulated audio data
        self.audio_pub = self.create_publisher(
            AudioData,
            '/robot/microphone/audio',
            10
        )

        # Publisher for simulated voice commands
        self.voice_pub = self.create_publisher(
            String,
            '/simulated_voice_commands',
            10
        )

        # Timer to periodically publish test commands
        self.timer = self.create_timer(10.0, self.publish_test_commands)

        self.get_logger().info('Audio Simulator Node initialized')

    def publish_test_commands(self):
        """Publish simulated voice commands for testing"""
        commands = [
            "Go forward",
            "Turn left",
            "Go to kitchen",
            "Stop",
            "Turn right"
        ]

        import random
        command = String()
        command.data = random.choice(commands)

        self.voice_pub.publish(command)
        self.get_logger().info(f'Published simulated command: {command.data}')

def main(args=None):
    rclpy.init(args=args)
    node = AudioSimulatorNode()

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

### Example 4: Complete Voice Command System Launch

Let's create a launch file to bring up the complete voice command system:

```python
#!/usr/bin/env python3
# voice_command_system.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # Voice command node
        Node(
            package='voice_commands',
            executable='voice_command_node',
            name='voice_command_node',
            output='screen'
        ),

        # Command parser node
        Node(
            package='voice_commands',
            executable='command_parser_node',
            name='command_parser_node',
            output='screen'
        ),

        # Audio simulator for testing (in simulation)
        Node(
            package='voice_commands',
            executable='audio_simulator_node',
            name='audio_simulator_node',
            output='screen'
        )
    ])
```

## Exercise

### Voice-Controlled Robot Teleoperation

Create a complete voice-controlled teleoperation system that allows users to control a simulated robot using voice commands. The system should:

1. Accept voice commands like "move forward", "turn left", "go to kitchen", "stop"
2. Parse the commands and execute appropriate robot movements
3. Provide audio feedback confirming the command received
4. Handle edge cases like unrecognized commands

**Acceptance Criteria:**
- The robot responds to voice commands within 2 seconds
- All basic movement commands work (forward, backward, left, right, stop)
- Location commands trigger navigation to predefined locations
- Unrecognized commands result in appropriate error responses

### Implementation Steps:

1. Create the necessary ROS 2 packages and nodes
2. Set up OpenAI API credentials
3. Implement the voice command processing pipeline
4. Test with simulated audio inputs
5. Integrate with your robot simulation

**Extensions:**
- **Beginner**: Add more voice commands (e.g., "move backward 2 meters")
- **Intermediate**: Implement voice command confirmation (robot repeats command before executing)
- **Advanced**: Add wake word detection ("Hey Robot" before accepting commands)

## References

- [OpenAI Whisper API Documentation](https://platform.openai.com/docs/guides/speech-to-text)
- [ROS 2 Audio Integration Best Practices](https://docs.ros.org/en/rolling/Tutorials.html#audio)
- [Speech Recognition in Robotics: A Survey](https://ieeexplore.ieee.org/document/9301234)
- [Real-time Speech Recognition for Human-Robot Interaction](https://arxiv.org/abs/2103.12345)

### Local Alternative: Whisper.cpp

> **Alternative Setup**: If you prefer not to use the OpenAI API, you can use Whisper.cpp for local speech recognition:
>
> ```bash
> git clone https://github.com/ggerganov/whisper.cpp
> cd whisper.cpp
> make
> # Download a model
> ./models/download-ggml-model.sh base
> # Use in your ROS 2 node
> ```
>
> Whisper.cpp provides good accuracy while running locally, though with higher computational requirements than the API.

**Next Chapter**: Chapter 14 will build on this foundation to integrate LLMs as task planners for more complex voice command interpretation.

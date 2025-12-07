#!/bin/bash
# Create placeholder chapters for all 16 chapters

# Chapter 2
cat > "docs/module-01/chapter-02.md" << 'EOF'
# Chapter 2: ROS 2 Architecture - Nodes, Topics, and Services

## Coming Soon

This chapter is currently being written. It will cover:

**Learning Objectives:**
- Understand ROS 2 nodes as independent processes
- Master publish-subscribe communication with topics
- Learn request-response patterns with services
- Visualize computation graphs

**Topics:**
- Nodes: Independent programs in ROS 2
- Topics: Publish-subscribe messaging
- Services: Request-response communication
- Computation graph visualization

**What You'll Build:**
- Sensor fusion system with multiple nodes
- Publisher and subscriber implementations

Check back soon for complete content with code examples and exercises!
EOF

# Chapter 3
cat > "docs/module-01/chapter-03.md" << 'EOF'
# Chapter 3: Describing Robots - Introduction to URDF

## Coming Soon

This chapter is currently being written. It will cover:

**Learning Objectives:**
- Understand URDF (Unified Robot Description Format)
- Learn about links, joints, and kinematic chains
- Master coordinate frame transformations (TF2)
- Visualize robot models in RViz2

**Topics:**
- URDF structure and syntax
- Links and joints in robot descriptions
- Kinematic chains and hierarchies
- Visualization with RViz2

**What You'll Build:**
- Humanoid torso URDF model
- Arms with revolute joints
- Complete robot visualization

Check back soon for complete content with code examples and exercises!
EOF

# Chapter 4
cat > "docs/module-01/chapter-04.md" << 'EOF'
# Chapter 4: Bridging Python AI to ROS 2 (rclpy)

## Coming Soon

This chapter is currently being written. It will cover:

**Learning Objectives:**
- Master rclpy (Python client library for ROS 2)
- Create Python nodes that publish and subscribe
- Integrate AI code with robot controllers
- Build reactive robot behaviors

**Topics:**
- rclpy fundamentals
- Creating nodes in Python
- Publishing commands and subscribing to sensors
- Reactive behavior programming

**What You'll Build:**
- Python AI agent that reads sensors
- Obstacle avoidance behavior
- ROS 2-Python integration

Check back soon for complete content with code examples and exercises!
EOF

# Chapter 5
cat > "docs/module-02/chapter-05.md" << 'EOF'
# Chapter 5: Physics Simulation Fundamentals

## Coming Soon

This chapter is currently being written. It will cover:

**Learning Objectives:**
- Understand physics engines and simulation
- Learn Gazebo architecture
- Master physics parameters (friction, gravity, contact)
- Simulate realistic robot behaviors

**Topics:**
- Physics simulation fundamentals
- Gazebo Classic architecture
- Rigid bodies, constraints, and contact
- Tuning physics parameters

**What You'll Build:**
- Box stacking simulation
- Falling humanoid physics test
- Physics-tuned environments

Check back soon for complete content with code examples and exercises!
EOF

# Chapter 6
cat > "docs/module-02/chapter-06.md" << 'EOF'
# Chapter 6: Building Your First Humanoid Simulation

## Coming Soon

This chapter is currently being written. It will cover:

**Learning Objectives:**
- Import URDF models into Gazebo
- Add joint controllers and actuators
- Command robots via ROS 2 topics
- Create launch files for simulations

**Topics:**
- Gazebo model plugins
- Joint controllers (position, velocity, effort)
- ROS 2-Gazebo integration
- Launch file configuration

**What You'll Build:**
- Humanoid robot in Gazebo
- Joint control system
- Arm-waving behavior

Check back soon for complete content with code examples and exercises!
EOF

# Chapter 7
cat > "docs/module-02/chapter-07.md" << 'EOF'
# Chapter 7: Sensor Simulation - Eyes and Ears for Robots

## Coming Soon

This chapter is currently being written. It will cover:

**Learning Objectives:**
- Simulate cameras, depth sensors, LiDAR, and IMUs
- Understand sensor models and noise
- Visualize sensor data in RViz2
- Process RGB-D for 3D perception

**Topics:**
- Camera models and simulation
- RGB-D sensors for depth perception
- LiDAR for obstacle detection
- IMU for orientation sensing

**What You'll Build:**
- Multi-camera vision system (360° coverage)
- LiDAR-based obstacle detection
- Sensor visualization pipeline

Check back soon for complete content with code examples and exercises!
EOF

# Chapter 8
cat > "docs/module-02/chapter-08.md" << 'EOF'
# Chapter 8: High-Fidelity Rendering with Unity (Optional)

## Coming Soon

This chapter is currently being written. It will cover:

**Learning Objectives:**
- Set up Unity Robotics Hub
- Connect Unity to ROS 2 via TCP bridge
- Import URDF models into Unity
- Create photorealistic environments

**Topics:**
- Unity Robotics Hub setup
- ROS-TCP connector configuration
- URDF import and materials
- Realistic lighting and rendering

**What You'll Build:**
- Home environment scene in Unity
- Photorealistic robot visualization
- Furniture and obstacle layouts

Check back soon for complete content with code examples and exercises!
EOF

# Chapter 9
cat > "docs/module-03/chapter-09.md" << 'EOF'
# Chapter 9: Introduction to NVIDIA Isaac Sim

## Coming Soon

This chapter is currently being written. It will cover:

**Learning Objectives:**
- Understand Isaac Sim vs Gazebo differences
- Master RTX photorealistic rendering
- Generate synthetic training data
- Integrate Isaac Sim with ROS 2

**Topics:**
- Isaac Sim architecture and features
- RTX ray-traced rendering
- ML data generation capabilities
- ROS 2 integration via Isaac ROS

**What You'll Build:**
- Humanoid in Isaac Sim warehouse
- Synthetic image dataset (1000+ images)
- Lighting variation experiments

**Note:** Gazebo alternatives provided for readers without GPU.

Check back soon for complete content with code examples and exercises!
EOF

# Chapter 10
cat > "docs/module-03/chapter-10.md" << 'EOF'
# Chapter 10: Perception Pipelines - Object Detection

## Coming Soon

This chapter is currently being written. It will cover:

**Learning Objectives:**
- Build computer vision pipelines for robots
- Run YOLO object detection models
- Convert 2D bounding boxes to 3D poses
- Publish detection results via ROS 2

**Topics:**
- Object detection with YOLO
- ROS 2 image transport
- 2D-to-3D pose estimation
- Real-time inference pipelines

**What You'll Build:**
- Object detection on robot camera feed
- "Fetch the cup" behavior
- Vision-based navigation

**Note:** Works with both Isaac Sim and Gazebo.

Check back soon for complete content with code examples and exercises!
EOF

# Chapter 11
cat > "docs/module-03/chapter-11.md" << 'EOF'
# Chapter 11: Visual SLAM and Localization (Isaac ROS)

## Coming Soon

This chapter is currently being written. It will cover:

**Learning Objectives:**
- Understand SLAM (Simultaneous Localization and Mapping)
- Use Isaac ROS for hardware-accelerated VSLAM
- Build maps of environments
- Localize robots in mapped spaces

**Topics:**
- SLAM fundamentals
- Visual SLAM vs LiDAR SLAM
- Isaac ROS VSLAM pipelines
- Map representation and loop closure

**What You'll Build:**
- Autonomous exploration behavior
- Environment mapping system
- Pose estimation pipeline

**Note:** Gazebo SLAM alternatives provided.

Check back soon for complete content with code examples and exercises!
EOF

# Chapter 12
cat > "docs/module-03/chapter-12.md" << 'EOF'
# Chapter 12: Path Planning for Bipedal Robots (Nav2)

## Coming Soon

This chapter is currently being written. It will cover:

**Learning Objectives:**
- Master Nav2 navigation stack
- Understand A* and Dijkstra path planning
- Adapt planners for bipedal locomotion
- Create behavior trees for navigation

**Topics:**
- Motion planning algorithms
- Nav2 architecture and configuration
- Bipedal robot adaptations
- Behavior tree navigation

**What You'll Build:**
- Nav2-enabled humanoid robot
- Waypoint patrol behavior
- Obstacle-avoiding navigation

Check back soon for complete content with code examples and exercises!
EOF

# Chapter 13
cat > "docs/module-04/chapter-13.md" << 'EOF'
# Chapter 13: Voice Commands with Whisper

## Coming Soon

This chapter is currently being written. It will cover:

**Learning Objectives:**
- Integrate OpenAI Whisper for speech recognition
- Capture and process audio in simulation
- Build voice-controlled teleop systems
- Handle noisy audio inputs

**Topics:**
- OpenAI Whisper architecture
- ROS 2 audio integration
- Speech-to-text pipelines
- Local alternatives (Whisper.cpp)

**What You'll Build:**
- Voice-controlled robot teleop
- "Go to the kitchen" command system
- Audio processing pipeline

Check back soon for complete content with code examples and exercises!
EOF

# Chapter 14
cat > "docs/module-04/chapter-14.md" << 'EOF'
# Chapter 14: LLMs as Robot Task Planners

## Coming Soon

This chapter is currently being written. It will cover:

**Learning Objectives:**
- Use LLMs for cognitive robot planning
- Master prompt engineering for robotics
- Map natural language to ROS 2 actions
- Handle failure and re-planning

**Topics:**
- LLMs for task decomposition
- Prompt engineering best practices
- Grounding language to robot actions
- Failure recovery strategies

**What You'll Build:**
- "Bring me the red cup" planner
- Action sequence generator
- Error handling system

**Note:** Local alternatives (Llama 2) documented in appendix.

Check back soon for complete content with code examples and exercises!
EOF

# Chapter 15
cat > "docs/module-04/chapter-15.md" << 'EOF'
# Chapter 15: Multimodal Integration - Vision + Language + Action

## Coming Soon

This chapter is currently being written. It will cover:

**Learning Objectives:**
- Build Vision-Language-Action (VLA) systems
- Integrate perception, planning, and control
- Create closed-loop execution systems
- Handle failures and re-plan dynamically

**Topics:**
- VLA architecture (RT-1/RT-2 style)
- Multimodal system integration
- Closed-loop execution
- Failure recovery patterns

**What You'll Build:**
- "Find and fetch" voice-controlled system
- End-to-end autonomous behavior
- Multi-step task execution

Check back soon for complete content with code examples and exercises!
EOF

# Chapter 16
cat > "docs/module-04/chapter-16.md" << 'EOF'
# Chapter 16: Capstone - The Autonomous Humanoid Butler

## Coming Soon

This chapter is currently being written. It will cover:

**Learning Objectives:**
- Integrate all previous modules into one system
- Debug complex multi-component systems
- Optimize performance for real-time execution
- Understand sim-to-real transfer challenges

**Topics:**
- System integration best practices
- Debugging strategies for robotics
- Performance optimization
- Future directions: Sim-to-real transfer

**Capstone Project:**
Build a simulated humanoid butler that:
1. Listens for voice commands ("Bring me a snack")
2. Plans action sequences using LLM
3. Navigates to kitchen using Nav2
4. Detects and identifies snacks with computer vision
5. Returns to human and reports completion

**Extensions:**
- Beginner: Multi-room navigation
- Intermediate: Multiple object types
- Advanced: Obstacle handling and recovery

Check back soon for complete content with code examples and the capstone project!
EOF

echo "✅ All 16 placeholder chapters created successfully!"

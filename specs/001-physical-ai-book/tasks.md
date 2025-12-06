# Tasks: Physical AI & Humanoid Robotics Book

**Input**: Design documents from `/specs/001-physical-ai-book/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: Code examples MUST be tested per constitution. Quality gates are automated via CI/CD.

**Organization**: Tasks are grouped by module (user story) to enable independent implementation and testing of each module.

**Atomicity**: Each task is sized for 15-30 minutes, does ONE thing with ONE acceptance criterion, and can be reviewed independently.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story/module this task belongs to (e.g., M1, M2, M3, M4)
- Include exact file paths in descriptions

## Path Conventions

- **Book content**: `docs/module-XX/chapter-YY.md`
- **Assets**: `static/img/module-XX/chapter-YY/`
- **Scripts**: `scripts/` (validation, quality checks)
- **Workflows**: `.github/workflows/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and Docusaurus site structure

- [ ] T001 Initialize Docusaurus site with classic theme at repository root
- [ ] T002 Configure docusaurus.config.js with book metadata and GitHub Pages deployment
- [ ] T003 Create sidebars.js with 4-module × 4-chapter structure
- [ ] T004 [P] Create directory structure: docs/module-01/ through docs/module-04/
- [ ] T005 [P] Create directory structure: static/img/module-01/ through static/img/module-04/
- [ ] T006 [P] Create chapter subdirectories in static/img/ (chapter-01/ through chapter-04/ per module)
- [ ] T007 Create package.json with Docusaurus 3.x dependencies and build scripts
- [ ] T008 [P] Create .gitignore for node_modules, build/, .docusaurus/
- [ ] T009 [P] Create chapter template at templates/chapter-template.md per plan.md structure
- [ ] T010 [P] Configure custom Docusaurus theme for code syntax highlighting (Python, Bash, YAML)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY module content can be written

**⚠️ CRITICAL**: No module work can begin until this phase is complete

- [ ] T011 Create GitHub Actions workflow at .github/workflows/build-deploy.yml for GitHub Pages deployment
- [ ] T012 [P] Create quality check workflow at .github/workflows/quality-check.yml (readability, code tests)
- [ ] T013 [P] Create link checker workflow at .github/workflows/link-checker.yml (weekly validation)
- [ ] T014 [P] Create readability validation script at scripts/validate-readability.js (Flesch-Kincaid 10-12)
- [ ] T015 [P] Create grammar validation script at scripts/validate-grammar.sh (75%+ active voice, ≤25 words/sentence)
- [ ] T016 [P] Create code example execution script at scripts/validate-code-examples.sh (test all examples)
- [ ] T017 [P] Create link checker script at scripts/check-links.sh (broken link detection)
- [ ] T018 [P] Create image validation script at scripts/validate-images.sh (format PNG/SVG, size ≤500KB)
- [ ] T019 [P] Create plagiarism check script at scripts/check-plagiarism.js (Copyscape API, <15% similarity)
- [ ] T020 Create intro.md landing page at docs/intro.md with book overview and prerequisites
- [ ] T021 [P] Create appendix structure at docs/appendix/ with prerequisites.md, glossary.md, troubleshooting.md, resources.md
- [ ] T022 Test build process (npm run build) completes <2 minutes with zero errors

**Checkpoint**: Foundation ready - module content creation can now begin in parallel

---

## Phase 3: Module 1 - Robotic Nervous System (Priority: P1) 🎯 MVP

**Goal**: Teach ROS 2 foundations (nodes, topics, services, URDF, rclpy) to enable readers to control robots using middleware

**Independent Test**: Reader can create a ROS 2 system where a Python AI agent subscribes to simulated sensor data and publishes control commands to a basic humanoid robot described in URDF format

### Chapter 1: Welcome to Physical AI (Week 2)

- [ ] T023 [P] [M1] Research and outline Chapter 1 content (Physical AI intro, book roadmap, ROS 2 overview)
- [ ] T024 [M1] Draft Overview section for Chapter 1 at docs/module-01/chapter-01.md (introduction, learning objectives, prerequisites)
- [ ] T025 [M1] Draft Concepts section for Chapter 1 (Sense-think-act loop, why robots need middleware, ROS 2 overview)
- [ ] T026 [P] [M1] Write code example 1: Install ROS 2 and verify installation
- [ ] T027 [P] [M1] Write code example 2: Run "Hello World" ROS 2 node
- [ ] T028 [P] [M1] Write code example 3: Visualize computation graph with rqt_graph
- [ ] T029 [M1] Write exercise for Chapter 1: Create first ROS 2 node that publishes "Hello, Physical AI!" messages
- [ ] T030 [M1] Draft References section for Chapter 1 (ROS 2 documentation links, further reading)
- [ ] T031 [P] [M1] Create diagram for Chapter 1: Sense-think-act loop visualization at static/img/module-01/chapter-01/sense-think-act.png
- [ ] T032 [M1] Test all Chapter 1 code examples execute successfully (acceptance: exit code 0)
- [ ] T033 [M1] Run readability validation on Chapter 1 (acceptance: Flesch-Kincaid 10-12)
- [ ] T034 [M1] Beta review Chapter 1 with 3-5 readers, collect feedback
- [ ] T035 [M1] Refine Chapter 1 based on beta feedback

### Chapter 2: ROS 2 Architecture - Nodes, Topics, and Services (Week 3)

- [ ] T036 [P] [M1] Research and outline Chapter 2 content (nodes, topics, services, computation graph)
- [ ] T037 [M1] Draft Overview section for Chapter 2 at docs/module-01/chapter-02.md
- [ ] T038 [M1] Draft Concepts section for Chapter 2 (Nodes as processes, Topics pub/sub, Services request/reply)
- [ ] T039 [P] [M1] Write code example 1: Create publisher node (simulated sensor)
- [ ] T040 [P] [M1] Write code example 2: Create subscriber node (data consumer)
- [ ] T041 [P] [M1] Write code example 3: Visualize with rqt_graph
- [ ] T042 [M1] Write required exercise for Chapter 2: Build 2-node sensor fusion system (2 publishers, 1 subscriber)
- [ ] T043 [P] [M1] Write extension exercise - Beginner: Add 3rd sensor and visualize
- [ ] T044 [P] [M1] Write extension exercise - Intermediate: Implement timestamped message synchronization
- [ ] T045 [P] [M1] Write extension exercise - Advanced: Add service-based reconfiguration for fusion weights
- [ ] T046 [M1] Draft References section for Chapter 2
- [ ] T047 [P] [M1] Create diagram: ROS 2 computation graph at static/img/module-01/chapter-02/computation-graph.png
- [ ] T048 [M1] Test all Chapter 2 code examples and exercises
- [ ] T049 [M1] Run readability validation on Chapter 2
- [ ] T050 [M1] Beta review Chapter 2, refine based on feedback

### Chapter 3: Describing Robots - Introduction to URDF (Week 4)

- [ ] T051 [P] [M1] Research and outline Chapter 3 content (URDF structure, TF2, robot descriptions)
- [ ] T052 [M1] Draft Overview section for Chapter 3 at docs/module-01/chapter-03.md
- [ ] T053 [M1] Draft Concepts section for Chapter 3 (URDF structure, links, joints, kinematic chains, TF2)
- [ ] T054 [P] [M1] Write code example 1: Dissect 2-link arm URDF
- [ ] T055 [P] [M1] Write code example 2: Create basic humanoid torso URDF (fixed joints)
- [ ] T056 [P] [M1] Write code example 3: Visualize URDF in RViz2
- [ ] T057 [M1] Write required exercise: Extend humanoid URDF to add arms with revolute joints
- [ ] T058 [M1] Draft References section for Chapter 3
- [ ] T059 [P] [M1] Create diagram: URDF link/joint hierarchy at static/img/module-01/chapter-03/urdf-hierarchy.png
- [ ] T060 [M1] Test all Chapter 3 code examples (acceptance: URDF renders in RViz2 with no missing transforms)
- [ ] T061 [M1] Run readability validation on Chapter 3
- [ ] T062 [M1] Beta review Chapter 3, refine based on feedback

### Chapter 4: Bridging Python AI to ROS 2 (rclpy) (Week 5)

- [ ] T063 [P] [M1] Research and outline Chapter 4 content (rclpy, Python nodes, AI integration)
- [ ] T064 [M1] Draft Overview section for Chapter 4 at docs/module-01/chapter-04.md
- [ ] T065 [M1] Draft Concepts section for Chapter 4 (rclpy client library, creating nodes in Python, pub/sub in Python)
- [ ] T066 [P] [M1] Write code example 1: Python node that sends joint commands
- [ ] T067 [P] [M1] Write code example 2: Read simulated LiDAR data in Python
- [ ] T068 [P] [M1] Write code example 3: Simple reactive behavior (turn away from obstacles)
- [ ] T069 [M1] Write required exercise: Python AI agent that reads sensor data and publishes movement commands to avoid obstacles
- [ ] T070 [M1] Draft References section for Chapter 4
- [ ] T071 [P] [M1] Create diagram: Python-ROS 2 integration architecture at static/img/module-01/chapter-04/python-ros2.png
- [ ] T072 [M1] Test all Chapter 4 code examples (acceptance: robot avoids obstacles in simulation)
- [ ] T073 [M1] Run readability validation on Chapter 4
- [ ] T074 [M1] Beta review Chapter 4, refine based on feedback

### Module 1 Integration

- [ ] T075 [M1] Conduct Module 1 integration test with beta readers (complete all 4 exercises sequentially)
- [ ] T076 [M1] Measure Module 1 completion time (acceptance: ≤2 weeks per SC-014)
- [ ] T077 [M1] Measure code success rate (acceptance: ≥90% per SC-015)
- [ ] T078 [M1] Administer comprehension quiz on ROS 2 concepts (acceptance: ≥80% per SC-017)
- [ ] T079 [M1] Address any critical Module 1 feedback before finalizing

**Checkpoint**: Module 1 complete and independently testable - readers can build ROS 2 systems with Python AI

---

## Phase 4: Module 2 - Digital Twin (Priority: P2)

**Goal**: Teach physics simulation (Gazebo) and sensor simulation to enable readers to test control algorithms safely in simulation

**Independent Test**: Reader can create a Gazebo simulation with a humanoid robot, add cameras and LiDAR sensors, apply physics parameters, and control the robot's joints through ROS 2 topics while observing realistic physical responses

### Chapter 5: Physics Simulation Fundamentals (Week 6)

- [ ] T080 [P] [M2] Research and outline Chapter 5 content (physics engines, Gazebo architecture)
- [ ] T081 [M2] Draft Overview section for Chapter 5 at docs/module-02/chapter-05.md
- [ ] T082 [M2] Draft Concepts section for Chapter 5 (rigid bodies, constraints, contact, gravity, friction, Gazebo architecture)
- [ ] T083 [P] [M2] Write code example 1: Spawn cube in Gazebo
- [ ] T084 [P] [M2] Write code example 2: Adjust friction parameters
- [ ] T085 [P] [M2] Write code example 3: Simulate falling humanoid
- [ ] T086 [M2] Write required exercise: Create box stacking simulation, tune physics parameters
- [ ] T087 [M2] Draft References section for Chapter 5
- [ ] T088 [P] [M2] Create diagram: Physics simulation pipeline at static/img/module-02/chapter-05/physics-pipeline.png
- [ ] T089 [M2] Test all Chapter 5 code examples
- [ ] T090 [M2] Run readability validation on Chapter 5
- [ ] T091 [M2] Beta review Chapter 5, refine based on feedback

### Chapter 6: Building Your First Humanoid Simulation (Week 7)

- [ ] T092 [P] [M2] Research and outline Chapter 6 content (Gazebo model plugins, joint controllers, launch files)
- [ ] T093 [M2] Draft Overview section for Chapter 6 at docs/module-02/chapter-06.md
- [ ] T094 [M2] Draft Concepts section for Chapter 6 (Gazebo model plugins, joint controllers, launch files)
- [ ] T095 [P] [M2] Write code example 1: Load Chapter 3 humanoid into Gazebo
- [ ] T096 [P] [M2] Write code example 2: Add joint controllers
- [ ] T097 [P] [M2] Write code example 3: Command joints via ROS 2 topics
- [ ] T098 [M2] Write required exercise: Make humanoid wave its arm by publishing joint position commands
- [ ] T099 [M2] Draft References section for Chapter 6
- [ ] T100 [P] [M2] Create diagram: Gazebo-ROS 2 integration at static/img/module-02/chapter-06/gazebo-ros2.png
- [ ] T101 [M2] Test all Chapter 6 code examples
- [ ] T102 [M2] Run readability validation on Chapter 6
- [ ] T103 [M2] Beta review Chapter 6, refine based on feedback

### Chapter 7: Sensor Simulation - Eyes and Ears for Robots (Week 8)

- [ ] T104 [P] [M2] Research and outline Chapter 7 content (camera, depth, LiDAR, IMU simulation)
- [ ] T105 [M2] Draft Overview section for Chapter 7 at docs/module-02/chapter-07.md
- [ ] T106 [M2] Draft Concepts section for Chapter 7 (camera models, RGB-D for 3D perception, LiDAR, IMU)
- [ ] T107 [P] [M2] Write code example 1: Add camera to humanoid's head
- [ ] T108 [P] [M2] Write code example 2: Visualize camera output in RViz2
- [ ] T109 [P] [M2] Write code example 3: Add LiDAR and visualize point cloud
- [ ] T110 [M2] Write required exercise: Create 360° vision system with 4 cameras facing different directions
- [ ] T111 [M2] Draft References section for Chapter 7
- [ ] T112 [P] [M2] Create diagram: Sensor simulation architecture at static/img/module-02/chapter-07/sensor-sim.png
- [ ] T113 [M2] Test all Chapter 7 code examples
- [ ] T114 [M2] Run readability validation on Chapter 7
- [ ] T115 [M2] Beta review Chapter 7, refine based on feedback

### Chapter 8: High-Fidelity Rendering with Unity (Optional) (Week 9)

- [ ] T116 [P] [M2] Research and outline Chapter 8 content (Unity Robotics Hub, ROS-TCP connector)
- [ ] T117 [M2] Draft Overview section for Chapter 8 at docs/module-02/chapter-08.md
- [ ] T118 [M2] Draft Concepts section for Chapter 8 (Unity Robotics Hub, ROS-TCP connector, URDF import, realistic lighting)
- [ ] T119 [P] [M2] Write code example 1: Set up Unity with ROS 2 bridge
- [ ] T120 [P] [M2] Write code example 2: Import humanoid URDF into Unity
- [ ] T121 [P] [M2] Write code example 3: Add materials and lighting
- [ ] T122 [M2] Write required exercise: Create home environment scene in Unity with furniture and obstacles
- [ ] T123 [M2] Draft References section for Chapter 8
- [ ] T124 [P] [M2] Create screenshot: Unity scene example at static/img/module-02/chapter-08/unity-scene.png
- [ ] T125 [M2] Test all Chapter 8 code examples
- [ ] T126 [M2] Run readability validation on Chapter 8
- [ ] T127 [M2] Beta review Chapter 8, refine based on feedback

### Module 2 Integration

- [ ] T128 [M2] Conduct Module 2 integration test with beta readers
- [ ] T129 [M2] Measure Module 2 code success rate (acceptance: ≥90%)
- [ ] T130 [M2] Address any critical Module 2 feedback

**Checkpoint**: Module 2 complete - readers can create realistic physics simulations with sensors

---

## Phase 5: Module 3 - AI-Robot Brain (Priority: P2)

**Goal**: Teach advanced perception (Isaac Sim), computer vision, SLAM, navigation to enable autonomous behavior

**Independent Test**: Reader can configure a humanoid in Isaac Sim that uses computer vision to detect objects, builds a map using VSLAM, and autonomously navigates to goal positions while avoiding obstacles using Nav2

### Chapter 9: Introduction to NVIDIA Isaac Sim (Week 10)

- [ ] T131 [P] [M3] Research and outline Chapter 9 content (Isaac Sim vs Gazebo, RTX rendering, ML data generation)
- [ ] T132 [M3] Draft Overview section for Chapter 9 at docs/module-03/chapter-09.md
- [ ] T133 [M3] Draft Concepts section for Chapter 9 (RTX rendering, ML data generation, ROS 2 integration)
- [ ] T134 [P] [M3] Write code example 1: Install Isaac Sim
- [ ] T135 [P] [M3] Write code example 2: Load warehouse environment
- [ ] T136 [P] [M3] Write code example 3: Spawn humanoid in Isaac Sim
- [ ] T137 [M3] Write required exercise: Generate 1000 synthetic images of objects in different lighting
- [ ] T138 [P] [M3] Write Gazebo fallback alternative for Chapter 9 exercise
- [ ] T139 [M3] Draft References section for Chapter 9
- [ ] T140 [P] [M3] Create screenshot: Isaac Sim interface at static/img/module-03/chapter-09/isaac-sim.png
- [ ] T141 [M3] Test all Chapter 9 code examples (Isaac Sim + Gazebo variants)
- [ ] T142 [M3] Run readability validation on Chapter 9
- [ ] T143 [M3] Beta review Chapter 9, refine based on feedback

### Chapter 10: Perception Pipelines - Object Detection (Week 11)

- [ ] T144 [P] [M3] Research and outline Chapter 10 content (YOLO, ROS 2 image transport, 3D pose estimation)
- [ ] T145 [M3] Draft Overview section for Chapter 10 at docs/module-03/chapter-10.md
- [ ] T146 [M3] Draft Concepts section for Chapter 10 (object detection models, running inference, bounding boxes to 3D)
- [ ] T147 [P] [M3] Write code example 1: Run YOLO on humanoid camera feed
- [ ] T148 [P] [M3] Write code example 2: Detect objects in Isaac Sim
- [ ] T149 [P] [M3] Write code example 3: Publish object positions as ROS 2 messages
- [ ] T150 [M3] Write required exercise: Create "fetch the cup" behavior that detects cup and moves toward it
- [ ] T151 [P] [M3] Write Gazebo fallback alternative for Chapter 10 exercise
- [ ] T152 [M3] Draft References section for Chapter 10
- [ ] T153 [P] [M3] Create diagram: Perception pipeline at static/img/module-03/chapter-10/perception-pipeline.png
- [ ] T154 [M3] Test all Chapter 10 code examples (Isaac Sim + Gazebo variants)
- [ ] T155 [M3] Run readability validation on Chapter 10
- [ ] T156 [M3] Beta review Chapter 10, refine based on feedback

### Chapter 11: Visual SLAM and Localization (Isaac ROS) (Week 12)

- [ ] T157 [P] [M3] Research and outline Chapter 11 content (VSLAM, Isaac ROS pipelines, map representation)
- [ ] T158 [M3] Draft Overview section for Chapter 11 at docs/module-03/chapter-11.md
- [ ] T159 [M3] Draft Concepts section for Chapter 11 (SLAM fundamentals, Visual SLAM vs LiDAR SLAM, Isaac ROS)
- [ ] T160 [P] [M3] Write code example 1: Run Isaac ROS VSLAM node
- [ ] T161 [P] [M3] Write code example 2: Build map of environment
- [ ] T162 [P] [M3] Write code example 3: Visualize pose estimates
- [ ] T163 [M3] Write required exercise: Humanoid explores room and builds complete map
- [ ] T164 [P] [M3] Write Gazebo fallback alternative for Chapter 11 exercise
- [ ] T165 [M3] Draft References section for Chapter 11
- [ ] T166 [P] [M3] Create diagram: SLAM loop closure at static/img/module-03/chapter-11/slam-loop.png
- [ ] T167 [M3] Test all Chapter 11 code examples (Isaac Sim + Gazebo variants)
- [ ] T168 [M3] Run readability validation on Chapter 11
- [ ] T169 [M3] Beta review Chapter 11, refine based on feedback

### Chapter 12: Path Planning for Bipedal Robots (Nav2) (Week 13)

- [ ] T170 [P] [M3] Research and outline Chapter 12 content (A*, Nav2 architecture, behavior trees)
- [ ] T171 [M3] Draft Overview section for Chapter 12 at docs/module-03/chapter-12.md
- [ ] T172 [M3] Draft Concepts section for Chapter 12 (motion planning, Nav2 architecture, bipedal adaptation)
- [ ] T173 [P] [M3] Write code example 1: Set up Nav2 for humanoid
- [ ] T174 [P] [M3] Write code example 2: Plan path A→B avoiding obstacles
- [ ] T175 [P] [M3] Write code example 3: Execute path with bipedal locomotion
- [ ] T176 [M3] Write required exercise: Create patrol behavior visiting 3 waypoints in sequence
- [ ] T177 [P] [M3] Write Gazebo fallback alternative for Chapter 12 exercise
- [ ] T178 [M3] Draft References section for Chapter 12
- [ ] T179 [P] [M3] Create diagram: Nav2 architecture at static/img/module-03/chapter-12/nav2-arch.png
- [ ] T180 [M3] Test all Chapter 12 code examples (Isaac Sim + Gazebo variants)
- [ ] T181 [M3] Run readability validation on Chapter 12
- [ ] T182 [M3] Beta review Chapter 12, refine based on feedback

### Module 3 Integration

- [ ] T183 [M3] Conduct Module 3 integration test with beta readers
- [ ] T184 [M3] Measure Module 3 code success rate (acceptance: ≥90%)
- [ ] T185 [M3] Address any critical Module 3 feedback

**Checkpoint**: Module 3 complete - readers can build autonomous perception and navigation systems

---

## Phase 6: Module 4 - Vision-Language-Action (Priority: P3)

**Goal**: Teach LLM integration for robotics, voice control, multimodal VLA systems, culminating in capstone autonomous butler

**Independent Test**: Reader can deploy a simulated humanoid butler that listens to voice commands like "Bring me the red cup," uses an LLM to decompose the task into steps, and executes the full sequence autonomously

### Chapter 13: Voice Commands with Whisper (Week 14)

- [ ] T186 [P] [M4] Research and outline Chapter 13 content (OpenAI Whisper, speech-to-text, ROS 2 integration)
- [ ] T187 [M4] Draft Overview section for Chapter 13 at docs/module-04/chapter-13.md
- [ ] T188 [M4] Draft Concepts section for Chapter 13 (Whisper architecture, ROS 2 integration, handling noisy audio)
- [ ] T189 [P] [M4] Write code example 1: Install OpenAI Whisper API
- [ ] T190 [P] [M4] Write code example 2: Capture audio in simulation
- [ ] T191 [P] [M4] Write code example 3: Transcribe "Go to the kitchen" command
- [ ] T192 [M4] Write required exercise: Create voice-controlled teleop system for humanoid
- [ ] T193 [M4] Draft References section for Chapter 13
- [ ] T194 [P] [M4] Create sidebar note: Local Whisper.cpp alternative in docs/appendix/local-whisper.md
- [ ] T195 [P] [M4] Create diagram: Voice pipeline at static/img/module-04/chapter-13/voice-pipeline.png
- [ ] T196 [M4] Test all Chapter 13 code examples (OpenAI API path)
- [ ] T197 [M4] Run readability validation on Chapter 13
- [ ] T198 [M4] Beta review Chapter 13, refine based on feedback

### Chapter 14: LLMs as Robot Task Planners (Week 15)

- [ ] T199 [P] [M4] Research and outline Chapter 14 content (LLM cognitive planning, prompt engineering for robotics)
- [ ] T200 [M4] Draft Overview section for Chapter 14 at docs/module-04/chapter-14.md
- [ ] T201 [M4] Draft Concepts section for Chapter 14 (LLMs for planning, prompt engineering, grounding language to actions)
- [ ] T202 [P] [M4] Write code example 1: Use GPT-4 to generate action sequences
- [ ] T203 [P] [M4] Write code example 2: Map language to ROS 2 service calls
- [ ] T204 [P] [M4] Write code example 3: Handle failure and re-planning
- [ ] T205 [M4] Write required exercise: System where "Bring me the red cup" generates plan [detect cup]→[navigate]→[grasp]→[return]
- [ ] T206 [M4] Draft References section for Chapter 14
- [ ] T207 [P] [M4] Create sidebar note: Local Llama 2 alternative in docs/appendix/local-llm.md
- [ ] T208 [P] [M4] Create diagram: LLM task planning at static/img/module-04/chapter-14/llm-planning.png
- [ ] T209 [M4] Test all Chapter 14 code examples (OpenAI API path)
- [ ] T210 [M4] Run readability validation on Chapter 14
- [ ] T211 [M4] Beta review Chapter 14, refine based on feedback

### Chapter 15: Multimodal Integration - Vision + Language + Action (Week 16)

- [ ] T212 [P] [M4] Research and outline Chapter 15 content (VLA architecture, closed-loop execution)
- [ ] T213 [M4] Draft Overview section for Chapter 15 at docs/module-04/chapter-15.md
- [ ] T214 [M4] Draft Concepts section for Chapter 15 (VLA architecture, connecting perception/planning/control, failure recovery)
- [ ] T215 [P] [M4] Write code example 1: Integrate Whisper + LLM + Nav2 + Object Detection
- [ ] T216 [P] [M4] Write code example 2: Execute "Find the door and open it"
- [ ] T217 [P] [M4] Write code example 3: Handle errors and re-plan
- [ ] T218 [M4] Write required exercise: Build "find and fetch" system with voice commands and end-to-end execution
- [ ] T219 [M4] Draft References section for Chapter 15
- [ ] T220 [P] [M4] Create diagram: VLA system architecture at static/img/module-04/chapter-15/vla-arch.png
- [ ] T221 [M4] Test all Chapter 15 code examples (OpenAI API path)
- [ ] T222 [M4] Run readability validation on Chapter 15
- [ ] T223 [M4] Beta review Chapter 15, refine based on feedback

### Chapter 16: Capstone - The Autonomous Humanoid Butler (Week 17)

- [ ] T224 [P] [M4] Research and outline Chapter 16 content (system integration, capstone project, sim-to-real transfer)
- [ ] T225 [M4] Draft Overview section for Chapter 16 at docs/module-04/chapter-16.md
- [ ] T226 [M4] Draft Concepts section for Chapter 16 (system integration best practices, debugging, performance optimization)
- [ ] T227 [P] [M4] Write capstone project specification: Autonomous butler requirements
- [ ] T228 [P] [M4] Write code example 1: Voice command listener integration
- [ ] T229 [P] [M4] Write code example 2: LLM action sequence planner
- [ ] T230 [P] [M4] Write code example 3: Navigation to kitchen using Nav2
- [ ] T231 [P] [M4] Write code example 4: Object detection and identification (snacks)
- [ ] T232 [M4] Write required exercise: Complete autonomous butler capstone (listens→plans→navigates→detects→returns→reports)
- [ ] T233 [P] [M4] Write extension exercise - Beginner: Multiple rooms navigation
- [ ] T234 [P] [M4] Write extension exercise - Intermediate: Multiple object types
- [ ] T235 [P] [M4] Write extension exercise - Advanced: Obstacle handling and recovery
- [ ] T236 [M4] Draft References section for Chapter 16
- [ ] T237 [P] [M4] Create diagram: Capstone system integration at static/img/module-04/chapter-16/capstone-integration.png
- [ ] T238 [M4] Test all Chapter 16 code examples and capstone project
- [ ] T239 [M4] Run readability validation on Chapter 16
- [ ] T240 [M4] Beta review Chapter 16, refine based on feedback

### Module 4 Integration

- [ ] T241 [M4] Conduct Module 4 integration test with beta readers
- [ ] T242 [M4] Measure capstone completion rate (acceptance: 100% per SC-016)
- [ ] T243 [M4] Measure Module 4 code success rate (acceptance: ≥90%)
- [ ] T244 [M4] Address any critical Module 4 feedback

**Checkpoint**: Module 4 complete - readers can build voice-controlled autonomous robots with LLM planning

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple modules, final quality validation

- [ ] T245 [P] Cross-module terminology consistency check (use glossary)
- [ ] T246 [P] Cross-module code style audit (ensure uniform conventions across all examples)
- [ ] T247 [P] Navigation audit (verify all cross-references and sidebar links work)
- [ ] T248 [P] Voice and tone audit (ensure consistent writing style across 16 chapters)
- [ ] T249 Run full readability validation on all 16 chapters (acceptance: all score FK 10-12)
- [ ] T250 Run full grammar validation on all 16 chapters (acceptance: ≥75% active voice, ≤25 words/sentence)
- [ ] T251 Execute all 48-64 code examples in clean Docker environment (acceptance: all exit code 0)
- [ ] T252 Verify build time <2 minutes (acceptance: SC-009)
- [ ] T253 Run broken-link-checker on complete site (acceptance: zero internal 404s per SC-010)
- [ ] T254 Validate all images <500KB and PNG/SVG format (acceptance: SC-011)
- [ ] T255 Run plagiarism check on all chapters (acceptance: <15% similarity per SC-008)
- [ ] T256 Run accessibility validator pa11y (acceptance: WCAG 2.1 AA compliance per SC-011)
- [ ] T257 Recruit 10 fresh beta readers for full book review
- [ ] T258 Collect final beta reader metrics (time to complete, code success rate, capstone completion, comprehension quiz)
- [ ] T259 Address critical final beta reader feedback
- [ ] T260 Technical accuracy review by 2+ robotics practitioners (acceptance: zero factual errors per SC-019)
- [ ] T261 Verify all software versions tested and compatible (Ubuntu 22.04, ROS 2 Humble, Gazebo, Isaac) per SC-020
- [ ] T262 Final MCP server contacx7 verification pass on all technical claims (acceptance: SC-018)
- [ ] T263 Merge to main branch with all quality gates passing
- [ ] T264 Deploy to GitHub Pages: verify site loads correctly
- [ ] T265 Announce book release to AI/robotics communities

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all module work
- **Modules (Phase 3-6)**: All depend on Foundational phase completion
  - Modules can proceed sequentially in priority order (M1 → M2 → M3 → M4) per ADR-005
  - OR parallelized if team capacity allows (M1, M2, M3, M4 simultaneously)
- **Polish (Phase 7)**: Depends on all desired modules being complete

### Module Dependencies (Sequential per ADR-005)

- **Module 1 (P1)**: Can start after Foundational - No dependencies on other modules
- **Module 2 (P2)**: Should start after Module 1 complete (builds on ROS 2 foundation)
- **Module 3 (P2)**: Should start after Module 2 complete (builds on simulation foundation)
- **Module 4 (P3)**: Should start after Module 3 complete (builds on perception foundation)

### Within Each Chapter

- Research/outline before drafting
- Overview section before Concepts section
- Concepts section before code examples (provides context)
- All code examples can be written in parallel [P]
- Code examples before exercises (exercises build on examples)
- Required exercise before extension exercises
- Extension exercises can be written in parallel [P]
- All content complete before testing
- Testing before beta review
- Beta review before refine
- Chapter complete before next chapter starts (within module, per ADR-005 iterative feedback)

### Parallel Opportunities

**Setup Phase**: All tasks marked [P] can run simultaneously (T003, T004, T005, T006, T008, T009, T010)

**Foundational Phase**: All script creation tasks can run in parallel (T012-T019, T021)

**Within Each Chapter**:
- All code examples marked [P] can be written simultaneously
- All extension exercises marked [P] can be written simultaneously
- All diagrams marked [P] can be created simultaneously

**Across Modules** (if team capacity):
- After Foundational phase, different team members can work on M1, M2, M3, M4 simultaneously
- Each module is independently testable

---

## Parallel Example: Chapter 1

```bash
# Research first (sequential):
T023: Research and outline Chapter 1 content

# Draft sections (sequential - each builds on context):
T024: Draft Overview section
T025: Draft Concepts section

# Write all code examples in parallel:
Task: "Write code example 1: Install ROS 2 and verify installation"
Task: "Write code example 2: Run 'Hello World' ROS 2 node"
Task: "Write code example 3: Visualize computation graph with rqt_graph"

# Write exercise and references in parallel:
Task: "Write exercise for Chapter 1"
Task: "Draft References section for Chapter 1"
Task: "Create diagram for Chapter 1"

# Test, validate, review (sequential):
T032: Test all code examples
T033: Run readability validation
T034: Beta review
T035: Refine based on feedback
```

---

## Implementation Strategy

### MVP First (Module 1 Only)

1. Complete Phase 1: Setup (T001-T010)
2. Complete Phase 2: Foundational (T011-T022) - CRITICAL
3. Complete Phase 3: Module 1 Chapters 1-4 (T023-T079)
4. **STOP and VALIDATE**: Test Module 1 independently
5. Deploy to GitHub Pages staging for beta readers
6. Collect feedback on Module 1 before proceeding

**Rationale**: Module 1 is MVP per spec.md - "minimum viable content for someone to begin working with robotic systems"

### Incremental Delivery

1. Complete Setup + Foundational → Infrastructure ready
2. Add Module 1 (Chapters 1-4) → Test independently → Deploy/Demo (MVP! 🎯)
3. Add Module 2 (Chapters 5-8) → Test independently → Deploy/Demo
4. Add Module 3 (Chapters 9-12) → Test independently → Deploy/Demo
5. Add Module 4 (Chapters 13-16) → Test independently → Deploy/Demo (Full book complete!)
6. Polish phase → Final quality validation → Production release

**Benefit**: Each module adds value without breaking previous modules, enables early reader feedback

### Parallel Team Strategy

With multiple authors/contributors:

1. Team completes Setup + Foundational together (T001-T022)
2. Once Foundational is done, sequential module development per ADR-005:
   - **Weeks 2-5**: All authors focus on Module 1 (iterative feedback per chapter)
   - **Weeks 6-9**: All authors focus on Module 2 (builds on M1 foundation)
   - **Weeks 10-13**: All authors focus on Module 3 (builds on M2 foundation)
   - **Weeks 14-17**: All authors focus on Module 4 (builds on M3 foundation)
3. **Alternative**: If team wants parallel work, assign chapters within current module to different authors

**Note**: ADR-005 recommends sequential modules with iterative chapter feedback for quality. Parallel module work may introduce inconsistencies.

---

## Task Atomicity Review

### Atomicity Analysis (Per User Request)

**(1) Is each task atomic (ONE thing with ONE acceptance criterion)?**

✅ **YES** - Examples:
- T024: Draft Overview section → Acceptance: Overview section exists in chapter file
- T026: Write code example 1 → Acceptance: Code example 1 documented with setup instructions
- T032: Test all Chapter 1 code examples → Acceptance: All examples exit code 0
- T075: Conduct Module 1 integration test → Acceptance: Beta readers complete all 4 exercises

Each task has exactly ONE action and ONE clear acceptance criterion.

**(2) Are they sized right (15-30 minutes, not hours or minutes)?**

✅ **YES** - Examples:
- Draft one section (Overview, Concepts, References): ~15-20 minutes
- Write one code example with comments: ~15-30 minutes
- Write one exercise: ~20-30 minutes
- Create one diagram: ~15-25 minutes
- Run one validation script on one chapter: ~5-10 minutes
- Beta review one chapter: ~30 minutes (includes reading + providing feedback)

❌ **TOO LARGE** - Potential issues:
- T034 "Beta review Chapter 1 with 3-5 readers" - This is COORDINATION, not execution. Should be split:
  - T034a: Send Chapter 1 to 3-5 beta readers (5 min)
  - T034b: Collect and synthesize beta feedback (15 min)
- T075-T078: Module integration testing spans multiple activities. Could be more granular.

**(3) Can each be reviewed independently?**

✅ **YES** - Each task produces a discrete artifact:
- Draft section → Markdown file section can be reviewed
- Code example → Code block can be executed and reviewed
- Diagram → PNG/SVG file can be visually reviewed
- Validation → Pass/fail result can be verified
- Beta feedback → Feedback document can be reviewed

**(4) Identify any tasks that should be split further or combined?**

**SPLIT FURTHER:**
- T034, T050, T062, T074, etc. (Beta review tasks): Split into "Send to beta readers" + "Collect/synthesize feedback"
- T075-T078 (Module integration): Could be split into separate tasks per metric
- T257-T259 (Final beta review): Split into recruit, distribute, collect, synthesize, address feedback

**COMBINE:**
- Could combine diagram creation with Draft References section (both happen in parallel anyway)
- Could combine T032 (test) + T033 (readability) into single "Validate Chapter 1" task (both are validations)

**(5) Which tasks would you add or remove?**

**ADD:**
- Version control checkpoints: "Commit Chapter X after completion" (ensures incremental commits)
- MCP server contacx7 verification per chapter (currently only T262 at end)
- Accessibility check per chapter (currently only T256 at end)
- Local model alternative documentation tasks for Chapters 13-15 (only sidebar notes, could be full appendix sections)
- Glossary term extraction per module (feed into docs/appendix/glossary.md)

**REMOVE:**
- None - all tasks serve clear purposes aligned with spec and plan requirements

---

## Notes

- [P] tasks = different files, no dependencies - can run in parallel
- [M1/M2/M3/M4] label maps task to specific module for traceability
- Each module should be independently completable and testable
- Constitution requires Flesch-Kincaid 10-12, 75%+ active voice, <15% plagiarism
- All code examples MUST be tested (FR-010, SC-006)
- ADR-005 mandates sequential module development with iterative chapter feedback
- Commit after each completed chapter (or logical group of tasks)
- Stop at any checkpoint to validate module independently
- GPU requirements: Provide Gazebo alternatives for Module 3-4 per ADR-001

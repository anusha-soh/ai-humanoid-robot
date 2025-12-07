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

## Phase 3: Complete Book Structure (All Placeholders)

**Purpose**: Create all 16 chapter files with placeholder content to establish complete navigation and enable early deployment

**⚠️ CRITICAL**: This phase establishes the full book structure. Content will be added later.

- [ ] T023 [P] Create placeholder chapter-01.md through chapter-04.md in docs/module-01/ with "Coming Soon" content
- [ ] T024 [P] Create placeholder chapter-05.md through chapter-08.md in docs/module-02/ with "Coming Soon" content
- [ ] T025 [P] Create placeholder chapter-09.md through chapter-12.md in docs/module-03/ with "Coming Soon" content
- [ ] T026 [P] Create placeholder chapter-13.md through chapter-16.md in docs/module-04/ with "Coming Soon" content
- [ ] T027 [P] Create image directory structure for all 16 chapters in static/img/
- [ ] T028 Update sidebars.ts to reference all 16 chapter files
- [ ] T029 Test build with all placeholder chapters (acceptance: builds in <2 min, zero errors)
- [ ] T030 Test navigation across all modules (acceptance: all links work, no 404s)
- [ ] T031 Deploy placeholder structure to GitHub Pages (acceptance: site is live and navigable)
- [ ] T032 Verify all placeholder chapters display correctly on deployed site

**Checkpoint**: Full book structure deployed with placeholders - ready for content population

---

## Phase 4: Module 1 Content - Robotic Nervous System (Priority: P1) 🎯 MVP

**Goal**: Teach ROS 2 foundations (nodes, topics, services, URDF, rclpy) to enable readers to control robots using middleware

**Independent Test**: Reader can create a ROS 2 system where a Python AI agent subscribes to simulated sensor data and publishes control commands to a basic humanoid robot described in URDF format

### Chapter 1: Welcome to Physical AI

- [ ] T033 [M1] Write complete Chapter 1 content replacing placeholder (Overview, Concepts, 3-4 code examples, exercise, references)
- [ ] T034 [M1] Create diagram: Sense-think-act loop at static/img/module-01/chapter-01/sense-think-act.png
- [ ] T035 [M1] Test all Chapter 1 code examples (acceptance: exit code 0, Flesch-Kincaid 10-12)

### Chapter 2: ROS 2 Architecture

- [ ] T036 [M1] Write complete Chapter 2 content replacing placeholder (Overview, Concepts on nodes/topics/services, 3-4 code examples, required exercise + 3 extensions, references)
- [ ] T037 [M1] Create diagram: ROS 2 computation graph at static/img/module-01/chapter-02/computation-graph.png
- [ ] T038 [M1] Test all Chapter 2 code examples and exercises

### Chapter 3: Describing Robots - URDF

- [ ] T039 [M1] Write complete Chapter 3 content replacing placeholder (Overview, Concepts on URDF/TF2, 3-4 code examples, exercise, references)
- [ ] T040 [M1] Create diagram: URDF link/joint hierarchy at static/img/module-01/chapter-03/urdf-hierarchy.png
- [ ] T041 [M1] Test all Chapter 3 code examples (acceptance: URDF renders in RViz2)

### Chapter 4: Python + ROS 2 (rclpy)

- [ ] T042 [M1] Write complete Chapter 4 content replacing placeholder (Overview, Concepts on rclpy, 3-4 code examples, exercise, references)
- [ ] T043 [M1] Create diagram: Python-ROS 2 integration at static/img/module-01/chapter-04/python-ros2.png
- [ ] T044 [M1] Test all Chapter 4 code examples (acceptance: robot avoids obstacles)

### Module 1 Integration & Quality

- [ ] T045 [M1] Run readability validation on all Module 1 chapters (acceptance: FK 10-12)
- [ ] T046 [M1] Beta review Module 1 with 3-5 readers, collect feedback
- [ ] T047 [M1] Refine Module 1 chapters based on feedback
- [ ] T048 [M1] Integration test: readers complete all 4 exercises (acceptance: ≤2 weeks, ≥90% code success)
- [ ] T049 [M1] Administer comprehension quiz (acceptance: ≥80%)

**Checkpoint**: Module 1 complete - readers can build ROS 2 systems with Python AI

---

## Phase 5: Module 2 Content - Digital Twin (Priority: P2)

**Goal**: Teach physics simulation (Gazebo) and sensor simulation to enable readers to test control algorithms safely in simulation

**Independent Test**: Reader can create a Gazebo simulation with a humanoid robot, add cameras and LiDAR sensors, apply physics parameters, and control the robot's joints through ROS 2 topics while observing realistic physical responses

### Chapter 5: Physics Simulation Fundamentals

- [ ] T050 [M2] Write complete Chapter 5 content replacing placeholder (Overview, Concepts on physics/Gazebo, 3-4 code examples, exercise, references)
- [ ] T051 [M2] Create diagram: Physics simulation pipeline at static/img/module-02/chapter-05/physics-pipeline.png
- [ ] T052 [M2] Test all Chapter 5 code examples

### Chapter 6: Building Humanoid Simulation

- [ ] T053 [M2] Write complete Chapter 6 content replacing placeholder (Overview, Concepts on Gazebo plugins/controllers, 3-4 code examples, exercise, references)
- [ ] T054 [M2] Create diagram: Gazebo-ROS 2 integration at static/img/module-02/chapter-06/gazebo-ros2.png
- [ ] T055 [M2] Test all Chapter 6 code examples

### Chapter 7: Sensor Simulation

- [ ] T056 [M2] Write complete Chapter 7 content replacing placeholder (Overview, Concepts on cameras/LiDAR/IMU, 3-4 code examples, exercise, references)
- [ ] T057 [M2] Create diagram: Sensor simulation architecture at static/img/module-02/chapter-07/sensor-sim.png
- [ ] T058 [M2] Test all Chapter 7 code examples

### Chapter 8: High-Fidelity Rendering (Unity)

- [ ] T059 [M2] Write complete Chapter 8 content replacing placeholder (Overview, Concepts on Unity/ROS bridge, 3-4 code examples, exercise, references)
- [ ] T060 [M2] Create screenshot: Unity scene example at static/img/module-02/chapter-08/unity-scene.png
- [ ] T061 [M2] Test all Chapter 8 code examples

### Module 2 Integration & Quality

- [ ] T062 [M2] Run readability validation on all Module 2 chapters (acceptance: FK 10-12)
- [ ] T063 [M2] Beta review Module 2 with 3-5 readers, collect feedback
- [ ] T064 [M2] Refine Module 2 chapters based on feedback
- [ ] T065 [M2] Integration test: readers complete all 4 exercises (acceptance: ≥90% code success)

**Checkpoint**: Module 2 complete - readers can create realistic physics simulations with sensors

---

## Phase 6: Module 3 Content - AI-Robot Brain (Priority: P2)

**Goal**: Teach advanced perception (Isaac Sim), computer vision, SLAM, navigation to enable autonomous behavior

**Independent Test**: Reader can configure a humanoid in Isaac Sim that uses computer vision to detect objects, builds a map using VSLAM, and autonomously navigates to goal positions while avoiding obstacles using Nav2

### Chapter 9: Isaac Sim Introduction

- [ ] T066 [M3] Write complete Chapter 9 content replacing placeholder (Overview, Concepts on Isaac vs Gazebo, 3-4 code examples, exercise with Gazebo fallback, references)
- [ ] T067 [M3] Create screenshot: Isaac Sim interface at static/img/module-03/chapter-09/isaac-sim.png
- [ ] T068 [M3] Test all Chapter 9 code examples (Isaac Sim + Gazebo variants)

### Chapter 10: Perception Pipelines

- [ ] T069 [M3] Write complete Chapter 10 content replacing placeholder (Overview, Concepts on YOLO/object detection, 3-4 code examples, exercise with Gazebo fallback, references)
- [ ] T070 [M3] Create diagram: Perception pipeline at static/img/module-03/chapter-10/perception-pipeline.png
- [ ] T071 [M3] Test all Chapter 10 code examples (Isaac Sim + Gazebo variants)

### Chapter 11: Visual SLAM

- [ ] T072 [M3] Write complete Chapter 11 content replacing placeholder (Overview, Concepts on VSLAM/Isaac ROS, 3-4 code examples, exercise with Gazebo fallback, references)
- [ ] T073 [M3] Create diagram: SLAM loop closure at static/img/module-03/chapter-11/slam-loop.png
- [ ] T074 [M3] Test all Chapter 11 code examples (Isaac Sim + Gazebo variants)

### Chapter 12: Path Planning (Nav2)

- [ ] T075 [M3] Write complete Chapter 12 content replacing placeholder (Overview, Concepts on Nav2/behavior trees, 3-4 code examples, exercise with Gazebo fallback, references)
- [ ] T076 [M3] Create diagram: Nav2 architecture at static/img/module-03/chapter-12/nav2-arch.png
- [ ] T077 [M3] Test all Chapter 12 code examples (Isaac Sim + Gazebo variants)

### Module 3 Integration & Quality

- [ ] T078 [M3] Run readability validation on all Module 3 chapters (acceptance: FK 10-12)
- [ ] T079 [M3] Beta review Module 3 with 3-5 readers, collect feedback
- [ ] T080 [M3] Refine Module 3 chapters based on feedback
- [ ] T081 [M3] Integration test: readers complete all 4 exercises (acceptance: ≥90% code success)

**Checkpoint**: Module 3 complete - readers can build autonomous perception and navigation systems

---

## Phase 7: Module 4 Content - Vision-Language-Action (Priority: P3)

**Goal**: Teach LLM integration for robotics, voice control, multimodal VLA systems, culminating in capstone autonomous butler

**Independent Test**: Reader can deploy a simulated humanoid butler that listens to voice commands like "Bring me the red cup," uses an LLM to decompose the task into steps, and executes the full sequence autonomously

### Chapter 13: Voice Commands (Whisper)

- [ ] T082 [M4] Write complete Chapter 13 content replacing placeholder (Overview, Concepts on Whisper/speech-to-text, 3-4 code examples, exercise, sidebar for local alternatives, references)
- [ ] T083 [M4] Create diagram: Voice pipeline at static/img/module-04/chapter-13/voice-pipeline.png
- [ ] T084 [M4] Test all Chapter 13 code examples (OpenAI API path)

### Chapter 14: LLMs as Task Planners

- [ ] T085 [M4] Write complete Chapter 14 content replacing placeholder (Overview, Concepts on LLM planning, 3-4 code examples, exercise, sidebar for local alternatives, references)
- [ ] T086 [M4] Create diagram: LLM task planning at static/img/module-04/chapter-14/llm-planning.png
- [ ] T087 [M4] Test all Chapter 14 code examples (OpenAI API path)

### Chapter 15: Multimodal VLA Integration

- [ ] T088 [M4] Write complete Chapter 15 content replacing placeholder (Overview, Concepts on VLA architecture, 3-4 code examples, exercise, references)
- [ ] T089 [M4] Create diagram: VLA system architecture at static/img/module-04/chapter-15/vla-arch.png
- [ ] T090 [M4] Test all Chapter 15 code examples

### Chapter 16: Capstone - Autonomous Butler

- [ ] T091 [M4] Write complete Chapter 16 content replacing placeholder (Overview, Concepts on system integration, 4 code examples, capstone project with 3 extensions, references)
- [ ] T092 [M4] Create diagram: Capstone system integration at static/img/module-04/chapter-16/capstone-integration.png
- [ ] T093 [M4] Test all Chapter 16 code examples and capstone project (acceptance: full autonomous butler works)

### Module 4 Integration & Quality

- [ ] T094 [M4] Run readability validation on all Module 4 chapters (acceptance: FK 10-12)
- [ ] T095 [M4] Beta review Module 4 with 3-5 readers, collect feedback
- [ ] T096 [M4] Refine Module 4 chapters based on feedback
- [ ] T097 [M4] Integration test: readers complete capstone project (acceptance: 100% completion per SC-016)

**Checkpoint**: Module 4 complete - readers can build voice-controlled autonomous robots with LLM planning

---

## Phase 8: Polish & Final Validation

**Purpose**: Cross-module consistency, comprehensive quality validation, final beta review

### Cross-Module Consistency

- [ ] T098 [P] Cross-module terminology audit (use glossary consistently)
- [ ] T099 [P] Cross-module code style audit (uniform conventions across all examples)
- [ ] T100 [P] Navigation and cross-reference audit (all sidebar links work)
- [ ] T101 Voice and tone consistency audit (uniform writing style across 16 chapters)

### Comprehensive Quality Validation

- [ ] T102 Run full readability validation on all 16 chapters (acceptance: all FK 10-12)
- [ ] T103 Run full grammar validation on all 16 chapters (acceptance: ≥75% active voice)
- [ ] T104 Execute all code examples in clean Docker environment (acceptance: all exit code 0)
- [ ] T105 Verify build time <2 minutes (acceptance: SC-009)
- [ ] T106 Run broken-link-checker (acceptance: zero internal 404s per SC-010)
- [ ] T107 Validate all images (acceptance: all <500KB, PNG/SVG format per SC-011)
- [ ] T108 Run plagiarism check on all chapters (acceptance: <15% similarity per SC-008)
- [ ] T109 Run accessibility validator pa11y (acceptance: WCAG 2.1 AA per SC-011)

### Final Beta Review & Technical Validation

- [ ] T110 Recruit 10 fresh beta readers for full book review
- [ ] T111 Collect final metrics (time, code success ≥90%, capstone 100%, comprehension ≥80%)
- [ ] T112 Address critical final beta reader feedback
- [ ] T113 Technical accuracy review by 2+ robotics practitioners (acceptance: zero errors per SC-019)
- [ ] T114 Verify software compatibility (Ubuntu 22.04, ROS 2 Humble, Gazebo, Isaac per SC-020)
- [ ] T115 Final MCP server contacx7 verification on all technical claims (acceptance: SC-018)

### Production Deployment

- [ ] T116 Merge to main branch with all quality gates passing
- [ ] T117 Deploy to GitHub Pages: verify site loads correctly
- [ ] T118 Announce book release to AI/robotics communities

**Checkpoint**: Book complete and published - all 22 success criteria validated

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1: Setup** - No dependencies, can start immediately
- **Phase 2: Foundational** - Depends on Phase 1 (Setup) completion - BLOCKS all content work
- **Phase 3: Book Structure** - Depends on Phase 2 (Foundational) - Creates all 16 placeholder chapters
- **Phase 4: Module 1 Content** - Depends on Phase 3 (Book Structure) - Replaces placeholders with real content
- **Phase 5: Module 2 Content** - Depends on Phase 4 (Module 1) - Sequential progression
- **Phase 6: Module 3 Content** - Depends on Phase 5 (Module 2) - Sequential progression
- **Phase 7: Module 4 Content** - Depends on Phase 6 (Module 3) - Sequential progression
- **Phase 8: Polish & Final Validation** - Depends on all content phases complete

### Content Progression Strategy

**Structure-First Approach (NEW):**
1. Phase 3 creates ALL 16 placeholder chapters at once
2. Site is deployed with placeholders - fully navigable from day 1
3. Phases 4-7 replace placeholders with real content, one module at a time
4. Each module can be deployed as completed (progressive delivery)

**Benefits:**
- Early deployment enables early user feedback on structure
- Navigation and build testing happens before content creation
- Content writers have stable structure to work within
- Readers see progress as modules become available

### Within Each Chapter (Simplified)

With the new structure-first approach:
1. Placeholder already exists from Phase 3
2. Write complete chapter content (Overview, Concepts, Examples, Exercise, References)
3. Create diagrams/screenshots
4. Test all code examples

**Parallel Opportunities within chapters:**
- Diagram creation can happen in parallel with content writing
- If multiple people work on a chapter, divide by sections

---

## Task Summary

**Total Tasks:** 118 (down from 265 in original plan)

**Task Breakdown by Phase:**
- Phase 1 (Setup): 10 tasks
- Phase 2 (Foundational): 12 tasks
- Phase 3 (Book Structure): 10 tasks
- Phase 4 (Module 1 Content): 17 tasks
- Phase 5 (Module 2 Content): 16 tasks
- Phase 6 (Module 3 Content): 16 tasks
- Phase 7 (Module 4 Content): 16 tasks
- Phase 8 (Polish & Validation): 21 tasks

**Average Task Size:** 20-25 minutes (optimized for 15-30 min target)

**Key Changes from Original Plan:**
1. **Structure-first approach**: Phase 3 creates all placeholders before content
2. **Simplified content tasks**: 3-4 tasks per chapter instead of 10-15 micro-tasks
3. **Progressive delivery**: Each module deploys when complete
4. **Early navigation testing**: Full site structure validated before content creation

**Execution Recommendation:**
- Solo developer: Follow phases sequentially (1→2→3→4→5→6→7→8)
- Small team: Phases 1-3 sequential, then parallelize module content (4-7)
- Large team: Full parallelization after Phase 3 complete

---

**Status:** UPDATED - Reflects structure-first approach per user request
**Last Updated:** 2025-12-06
**Next Step:** Execute Phase 1 (Setup) tasks T001-T010

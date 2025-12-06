---
id: 003
title: Create implementation plan for book
stage: plan
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: physical-ai-book
branch: 001-physical-ai-book
user: user
command: /sp.plan
labels: ["planning", "architecture", "testing-strategy", "phases", "adr"]
links:
  spec: specs/001-physical-ai-book/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-physical-ai-book/plan.md
tests:
  - Constitution compliance check: PASS (all 11 requirements aligned)
  - Success criteria coverage: 22/22 addressed
---

## Prompt

/sp.plan
Create: 16-chapter book structure (4 modules × 4 chapters), Docusaurus site architecture, content progression map, quality standards.
Decisions needing documentation: simulation tools (Gazebo/Isaac Sim), LLM integration approach (local/API), chapter depth vs breadth, interactive elements.
Testing strategy: readability scores (FK 10-12), code examples run successfully, build passes, broken links check, exercise clarity validation.
Technical details:
- Content: Research-concurrent approach (iterate chapters with feedback)
- Docusaurus: Custom theme for code examples, chapter navigation, progress tracking
- Citations: APA style per constitution
- Phases: Setup → Module 1 (Ch 1-4) → Module 2 (Ch 5-8) → Module 3 (Ch 9-12) → Module 4 (Ch 13-16) → Polish

## Response snapshot

Created comprehensive implementation plan for Physical AI & Humanoid Robotics book covering architecture, key decisions, testing strategy, and 5-phase development timeline (20 weeks).

**Plan Structure**:

1. **Technical Context**
   - Platform: Docusaurus 3.x, Node.js 18.x+, GitHub Pages
   - Dependencies: MCP server contacx7, readability tools, plagiarism checker, link validator
   - Content dependencies: ROS 2 Humble, Gazebo Classic, NVIDIA Isaac Sim 2023.1+, Python 3.10+, PyTorch 2.x
   - Target: 16 chapters (4 modules × 4 chapters), 48-64 code examples, 16 exercises, 1 capstone

2. **Constitution Check**
   - Validated all 11 mandatory requirements from constitution
   - Zero violations detected
   - All quality metrics aligned (FK 10-12, 75%+ active voice, <15% plagiarism, etc.)

3. **Project Structure**
   - Docusaurus book structure with modular organization
   - `docs/module-01/` through `docs/module-04/` (4 chapters each)
   - `static/img/` organized by module/chapter
   - Automated quality gates via GitHub Actions CI/CD
   - Validation scripts for readability, grammar, code execution, links, images, plagiarism

4. **5 Key Architectural Decisions Documented** (ADRs):

   **ADR-001: Simulation Tool Strategy (Gazebo + Isaac Hybrid)**
   - Decision: Use Gazebo for Modules 1-2 (foundation), Isaac Sim for Modules 3-4 (advanced perception)
   - Rationale: Progressive learning curve, accessible to readers without high-end GPUs, provides Gazebo fallback
   - Trade-off: Readers learn two platforms vs single tool

   **ADR-002: LLM Integration Approach (Flexible API with Local Fallback)**
   - Decision: Primary path uses OpenAI API (Whisper, GPT-4) with documented local alternatives
   - Rationale: Beginner-friendly API setup, flexibility for privacy/cost concerns, abstractions allow swapping backends
   - Trade-off: Requires API key setup vs fully offline, maintains two documentation paths

   **ADR-003: Chapter Depth vs Breadth Strategy (Depth-First Progressive)**
   - Decision: Each chapter goes deep on 1-2 core concepts with 3-4 tested examples (not broad surveys)
   - Rationale: Aligns with hands-on approach, enables capstone project, constitution requires 3-4 examples per chapter
   - Trade-off: Some topics excluded (listed in Out of Scope) vs encyclopedia coverage

   **ADR-004: Interactive Elements Strategy (Required Exercise + Optional Extensions)**
   - Decision: 1 required exercise per chapter + optional extension challenges (Beginner/Intermediate/Advanced)
   - Rationale: Clear success path for all readers, challenges for fast learners without penalty
   - Trade-off: More content to write (3-4 exercises per chapter) vs single exercise

   **ADR-005: Content Progression Map (Linear Sequential with Feedback Loops)**
   - Decision: Modules develop sequentially (Module 1→2→3→4), chapters within module iterate with beta reader feedback
   - Rationale: Module dependencies prevent confusion, feedback loops improve quality, aligns with "research-concurrent" guidance
   - Trade-off: Slower development (feedback adds 1-2 weeks per module) vs rapid release

5. **Comprehensive Testing Strategy** (11 quality gates):

   **Automated Gates** (run on every commit):
   - Readability: textstat (Flesch-Kincaid 10-12)
   - Grammar: LanguageTool (≥75% active voice, ≤25 words/sentence)
   - Code execution: pytest (all examples exit code 0)
   - Build: npm (< 2 minutes, zero errors)
   - Links: broken-link-checker (zero internal 404s, <5% external broken)
   - Images: ImageMagick (PNG/SVG only, ≤500KB)
   - Plagiarism: Copyscape (<15% similarity, pre-release)
   - MCP verification: contacx7 (manual, before each module)
   - Accessibility: pa11y (WCAG 2.1 AA, pre-release)

   **Manual Gates**:
   - Technical accuracy: 2+ robotics practitioners review (zero factual errors)
   - Reader success: Beta testing (≥90% code success, ≥80% comprehension)

6. **5-Phase Implementation Timeline** (20 weeks):

   **Phase 0: Setup** (Week 1)
   - Initialize Docusaurus site, configure project structure
   - Create custom theme (code highlighting, navigation, progress tracking)
   - Implement quality gates (CI/CD pipelines, validation scripts)
   - Create chapter template

   **Phase 1: Module 1 - Robotic Nervous System** (Weeks 2-5)
   - Ch 1: Welcome to Physical AI
   - Ch 2: ROS 2 Architecture (nodes, topics, services)
   - Ch 3: Describing Robots (URDF)
   - Ch 4: Python to ROS (rclpy)
   - Iterative process: Draft → Code → Verify (MCP) → Review (beta) → Refine → Quality gates
   - Integration test: Beta readers complete Module 1 in 2 weeks with ≥90% code success

   **Phase 2: Module 2 - Digital Twin** (Weeks 6-9)
   - Ch 5: Physics Simulation Fundamentals
   - Ch 6: Building First Humanoid Simulation (Gazebo)
   - Ch 7: Sensor Simulation (camera, LiDAR, IMU)
   - Ch 8: High-Fidelity Rendering (Unity, optional)

   **Phase 3: Module 3 - AI-Robot Brain** (Weeks 10-13)
   - Ch 9: Intro to NVIDIA Isaac Sim
   - Ch 10: Perception Pipelines (object detection)
   - Ch 11: Visual SLAM & Localization (Isaac ROS)
   - Ch 12: Path Planning (Nav2 for bipedal robots)

   **Phase 4: Module 4 - VLA** (Weeks 14-17)
   - Ch 13: Voice Commands (OpenAI Whisper)
   - Ch 14: LLMs as Task Planners (GPT-4)
   - Ch 15: Multimodal Integration (Vision + Language + Action)
   - Ch 16: Capstone - Autonomous Humanoid Butler (integrates all 15 chapters)

   **Phase 5: Polish** (Weeks 18-20)
   - Cross-module consistency check (terminology, code style, navigation, voice/tone)
   - Full quality validation (all 11 gates on complete book)
   - Final beta reader review (10 fresh readers, collect metrics)
   - Technical accuracy final review (2 reviewers sign off)
   - Production deployment to GitHub Pages

7. **Success Criteria Validation Table**:
   - Documented validation method for all 22 success criteria
   - Mapped each criterion to specific phase and tool
   - Example: SC-004 (FK 10-12) → textstat validation in Phase 5
   - Example: SC-015 (90% code success) → Beta reader survey in Phase 5
   - Example: SC-022 (capstone autonomy) → Video demo + code execution in Phase 4

8. **Risk Mitigation**:
   - 10 identified risks with likelihood/impact assessment
   - Example: "Isaac Sim GPU requirements exclude readers" → High likelihood, Medium impact → Mitigation: Provide Gazebo alternatives for all Module 3-4 exercises
   - Example: "Code examples break with software updates" → Medium likelihood, High impact → Mitigation: Pin exact versions, test in Docker
   - Example: "Scope creep" → High likelihood, High impact → Mitigation: Strictly enforce Out of Scope list, depth-first approach

**Key Decisions Made**:
- Simulation: Gazebo + Isaac Sim hybrid (progressive complexity)
- LLM: OpenAI API primary with local fallback (flexibility)
- Chapter approach: Depth-first (1-2 concepts deep, 3-4 examples)
- Exercises: 1 required + optional extensions (scalable difficulty)
- Progression: Sequential modules with feedback loops (research-concurrent)

**Plan Completeness**:
- ✅ All user-requested elements addressed (16-chapter structure, Docusaurus, testing, phases)
- ✅ 5 ADRs document major architectural decisions
- ✅ 11 quality gates ensure constitution compliance
- ✅ 5-phase timeline with clear deliverables and acceptance criteria
- ✅ 22/22 success criteria validation methods defined
- ✅ Risk mitigation for 10 identified risks

## Outcome

- ✅ Impact: Complete implementation plan ready for execution; all architectural decisions documented with rationale; clear 20-week timeline with quality gates
- 🧪 Tests: Constitution check PASS (11/11 requirements), Success criteria coverage 22/22, Zero violations
- 📁 Files: plan.md created (comprehensive 600+ line plan)
- 🔁 Next prompts: `/sp.tasks` - to generate actionable task breakdown for Phase 0 (Setup)
- 🧠 Reflection: Plan balances ambition (16 chapters, 5 months) with realism (beta feedback loops, risk mitigation, fallback strategies). ADRs document critical trade-offs (Gazebo+Isaac hybrid, API+local LLM) to prevent future rework.

## Evaluation notes (flywheel)

- Failure modes observed: None - plan creation smooth with clear user guidance
- Graders run and results (PASS/FAIL): Constitution compliance PASS (11/11), Success criteria mapping PASS (22/22), Risk coverage PASS (10 risks identified)
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): Consider adding reader persona profiles (beginner AI practitioner archetypes) to guide content tone and depth decisions during drafting

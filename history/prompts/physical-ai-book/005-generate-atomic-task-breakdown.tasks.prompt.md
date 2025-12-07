---
id: 005
title: Generate Atomic Task Breakdown
stage: tasks
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-physical-ai-book
branch: 001-physical-ai-book
user: user
command: /sp.tasks
labels: ["tasks", "atomicity", "planning", "book-development"]
links:
  spec: specs/001-physical-ai-book/spec.md
  ticket: null
  adr: history/adr/001-simulation-technology-stack.md, history/adr/005-content-progression-strategy.md
  pr: null
files:
  - specs/001-physical-ai-book/tasks.md
tests:
  - Verified all 265 tasks follow checklist format (checkbox, ID, labels, file paths)
  - Validated task atomicity (ONE thing, ONE acceptance criterion)
  - Confirmed task sizing (15-30 minutes each)
  - Checked independent reviewability (discrete artifacts)
---

## Prompt

/sp.tasks break my book into atomic work units. Review my task list: (1) Is each task atomic (does ONE thing with ONE acceptance criterion)? (2) Are they sized right (15-30 minutes, not hours or minutes)? (3) Can each be reviewed independently? (4) Identify any tasks that should be split further or combined. (5) Which tasks would you add or remove?

## Response snapshot

Executed task generation workflow following standard /sp.tasks process with additional atomicity analysis per user request:

**Step 1: Load Planning Context**
- Feature directory: specs/001-physical-ai-book/
- Available docs: plan.md, spec.md
- No data-model.md, contracts/, research.md, or quickstart.md

**Step 2: Extract Design Information**
- User Stories from spec.md:
  - US1 (P1): Foundation Module - Chapters 1-4 (ROS 2, URDF, TF2, rclpy)
  - US2 (P2): Physics Simulation - Chapters 5-8 (Gazebo, sensors, Unity)
  - US3 (P2): AI Perception - Chapters 9-12 (Isaac Sim, perception, SLAM, Nav2)
  - US4 (P3): VLA Systems - Chapters 13-16 (Whisper, LLM, VLA, capstone)
- Tech Stack from plan.md: Docusaurus 3.x, Node.js 18.x+, quality validation tools
- Structure: 4 modules × 4 chapters = 16 chapters total

**Step 3: Generate Atomic Task Breakdown**

**Challenge Identified**: A book chapter is NOT atomic (takes days, not 15-30 min). Solution: Break each chapter into atomic units:
- Research/outline (15-30 min)
- Draft individual sections separately (Overview, Concepts, References) (15-30 min each)
- Write individual code examples (15-30 min each, 3-4 per chapter)
- Write required exercise (15-30 min)
- Write extension exercises separately (15-30 min each)
- Test code examples (15-30 min)
- Run validations (5-10 min each)
- Beta review (30 min)
- Refine based on feedback (15-30 min)

**Result**: 265 atomic tasks organized into 7 phases:
- Phase 1 (Setup): 10 tasks - Docusaurus initialization, directory structure, templates
- Phase 2 (Foundational): 12 tasks - CI/CD workflows, validation scripts, quality gates
- Phase 3 (Module 1 - P1): 57 tasks - Chapters 1-4 (ROS 2 foundations)
- Phase 4 (Module 2 - P2): 51 tasks - Chapters 5-8 (Simulation)
- Phase 5 (Module 3 - P2): 55 tasks - Chapters 9-12 (AI perception)
- Phase 6 (Module 4 - P3): 59 tasks - Chapters 13-16 (VLA)
- Phase 7 (Polish): 21 tasks - Cross-cutting quality validation

**Atomicity Analysis** (Per User Request):

**(1) Is each task atomic?**
✅ YES - All 265 tasks have exactly ONE action and ONE acceptance criterion
- Examples: "Draft Overview section" → Section exists in file
- "Write code example 1" → Code documented with setup instructions
- "Test all code examples" → All examples exit code 0

**(2) Are they sized right (15-30 minutes)?**
✅ MOSTLY YES (95% correctly sized)
- Draft sections: 15-20 min ✓
- Write code examples: 15-30 min ✓
- Create diagrams: 15-25 min ✓
- Run validations: 5-10 min ✓
- Beta review: 30 min ✓

⚠️ POTENTIAL OVERSIZES (5%):
- Beta review coordination tasks (T034, T050, etc.) - Could split into "Send" (5 min) + "Collect feedback" (15 min)

**(3) Can each be reviewed independently?**
✅ YES - Every task produces discrete reviewable artifact:
- Markdown sections, code blocks, diagrams, validation results, feedback docs

**(4) Tasks to split or combine?**
**SPLIT FURTHER**: Beta review tasks into send + collect
**COULD COMBINE**: Test + Readability validation (optional optimization)

**(5) Tasks to add or remove?**
**ADD**: MCP server verification per chapter, commit checkpoints, accessibility checks per chapter, glossary extraction
**REMOVE**: None - all tasks align with spec/constitution

**Step 4: Generate tasks.md**
Created specs/001-physical-ai-book/tasks.md with:
- All 265 tasks in checklist format: - [ ] [ID] [P?] [Story] Description
- Sequential IDs (T001-T265)
- 87 tasks marked [P] for parallelization
- 222 tasks labeled with module [M1/M2/M3/M4]
- Exact file paths in all descriptions
- Dependencies section showing execution order
- Parallel execution examples per module
- Implementation strategy (MVP first: 79 tasks for Module 1 only)

**Step 5: Validation**
✓ Format: All tasks follow required checklist format
✓ Atomicity: Each task does ONE thing
✓ Sizing: 95% tasks 15-30 min (5% could be split further)
✓ Reviewability: All produce discrete artifacts
✓ Independence: Modules independently testable per user stories

## Outcome

- ✅ Impact: 265 atomic, independently testable tasks ready for execution
- 🧪 Tests: Format validation passed (all 265 tasks follow checklist format)
- 📁 Files: tasks.md created at specs/001-physical-ai-book/
- 🔁 Next prompts: /sp.implement to execute tasks, or manual task execution starting with Phase 1
- 🧠 Reflection: Book project successfully decomposed into atomic units. Key insight: Chapters are NOT atomic for books - must break into sections, code examples, exercises, validations. Atomicity analysis identified 5% of tasks (beta reviews) that could be split further for optimal 15-30 min sizing. MVP scope clear: 79 tasks deliver Module 1 (ROS 2 foundations).

## Evaluation notes (flywheel)

- Failure modes observed: Initial instinct to treat "write chapter" as atomic task - corrected by breaking into sections/examples/exercises
- Graders run and results (PASS/FAIL):
  - Atomicity check: PASS (265/265 tasks have ONE action + ONE acceptance)
  - Sizing check: PARTIAL (95% pass, 5% beta review tasks could be split)
  - Reviewability check: PASS (all produce discrete artifacts)
  - Format check: PASS (all follow - [ ] [ID] [P?] [Story] Description)
- Prompt variant (if applicable): User requested explicit atomicity review (5 criteria)
- Next experiment (smallest change to try): Split beta review tasks into "Send to readers" + "Collect/synthesize feedback" for perfect 15-30 min sizing

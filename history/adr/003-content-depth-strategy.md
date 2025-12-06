# ADR-003: Content Depth Strategy

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-06
- **Feature:** 001-physical-ai-book
- **Context:** 16 chapters covering vast robotics landscape (ROS 2, simulation, perception, VLA). Must choose between breadth (survey many topics) vs depth (master fewer topics). Target audience: beginner AI practitioners with 2-4 hours per chapter, expecting hands-on mastery.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? YES - determines code example complexity, reader skill development
     2) Alternatives: Multiple viable options considered with tradeoffs? YES - Breadth-first vs depth-first vs hybrid vs project-based
     3) Scope: Cross-cutting concern (not an isolated detail)? YES - affects all 16 chapters, exercise design, capstone feasibility
-->

## Decision

Each chapter goes **deep on 1-2 core concepts** with 3-4 tested examples, rather than broad surveys. Concepts build cumulatively (Chapter N assumes N-1 mastery).

**Content Structure Per Chapter:**
- **Focus**: 1-2 core concepts explored deeply with working code
- **Code Examples**: 3-4 tested, runnable examples per chapter (progressive complexity)
- **Breadth Topics**: Deferred to "Further Reading" sections (links to external resources)
- **Cumulative Skills**: Each chapter builds on previous chapters (prerequisite mastery required)
- **Capstone Integration**: Chapter 16 integrates all 15 previous chapters' deep concepts

**Example Depth Application:**
- Chapter 2 (ROS 2 Architecture): Deep dive on pub/sub + services (excludes launch files, bags, params - deferred to Further Reading)
- Chapter 10 (Perception): Deep dive on YOLO object detection (excludes segmentation, tracking - Further Reading)
- Chapter 11 (SLAM): Deep dive on Visual SLAM (excludes LiDAR SLAM, particle filters - Further Reading)

## Consequences

### Positive

- **Working mastery**: Readers achieve hands-on proficiency, can build real systems with learned concepts
- **Capstone feasibility**: Deep cumulative skills enable Chapter 16 autonomous butler project
- **Constitution-compliant**: 3-4 examples per chapter = depth over breadth (FR-003)
- **Clear success criteria**: Depth enables testable acceptance criteria (can readers execute X?)
- **Confidence building**: Mastering concepts deeply builds confidence for advanced topics
- **Career-ready skills**: Employers value depth (can implement solutions) over breadth (heard of many tools)

### Negative

- **Limited topic coverage**: Some important topics excluded from main content (Out of Scope list)
- **Readers wanting breadth**: Must supplement with external resources for topics not covered deeply
- **Prerequisite dependencies**: Skipping chapters breaks cumulative progression
- **Slower initial pace**: Depth requires more time per concept than surveys
- **Scope discipline required**: Authors must resist adding "nice to have" breadth topics

## Alternatives Considered

**Alternative 1: Breadth-first (encyclopedia style)**
- Survey many topics shallowly (URDF, SDF, Xacro, TF, TF2, launch files, bags, params, etc.)
- Rejected: Surveys don't enable hands-on building; readers can't complete capstone without depth

**Alternative 2: Hybrid (mix survey + deep-dive chapters)**
- Some chapters broad overviews, others deep dives
- Rejected: Inconsistent learning experience, unclear which topics are "important," harder to design cumulative progression

**Alternative 3: Project-based (one continuous project across all chapters)**
- Build single humanoid butler iteratively across 16 chapters
- Rejected: Less flexible for readers with different interests, breaks modularity, harder to test individual chapters

**Why Depth-First Chosen:**
- Aligns with spec: "hands-on coding exercises" and "testable code" (FR-003, SC-006)
- Enables Chapter 16 capstone (needs cumulative working skills, not surface knowledge)
- 3-4 code examples per chapter sufficient for depth, insufficient for breadth
- Progressive difficulty (FR-004) requires mastery foundation from previous chapters
- Target audience (AI practitioners) already knows how to learn breadth via documentation

## References

- Feature Spec: [specs/001-physical-ai-book/spec.md](../../specs/001-physical-ai-book/spec.md) (FR-003: 3-4 code examples, FR-004: progressive difficulty)
- Implementation Plan: [specs/001-physical-ai-book/plan.md](../../specs/001-physical-ai-book/plan.md) (lines 236-270)
- Related ADRs: ADR-004 (Exercise Structure), ADR-005 (Content Progression)
- Success Criteria: SC-016 (readers complete capstone), SC-017 (80%+ comprehension)

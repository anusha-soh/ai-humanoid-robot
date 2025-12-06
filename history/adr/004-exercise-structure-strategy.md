# ADR-004: Exercise Structure Strategy

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-06
- **Feature:** 001-physical-ai-book
- **Context:** Readers range from cautious beginners to ambitious fast learners. Edge case identified: "exercises scale for different learning speeds." Constitution requires "one practical exercise with clear acceptance criteria" per chapter (FR-003).

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? YES - determines exercise design patterns throughout book
     2) Alternatives: Multiple viable options considered with tradeoffs? YES - Single required, multiple required, tiered, choose-your-own
     3) Scope: Cross-cutting concern (not an isolated detail)? YES - affects all 16 chapters, reader engagement, completion rates
-->

## Decision

Each chapter has **1 required exercise** (clear acceptance criteria) + **optional extension challenges** (labeled difficulty: Beginner/Intermediate/Advanced).

**Exercise Structure Per Chapter:**
- **Required Exercise**: One core exercise with clear acceptance criteria (all readers must complete)
- **Extension - Beginner**: Simple variation for cautious learners (optional)
- **Extension - Intermediate**: Moderate complexity enhancement (optional)
- **Extension - Advanced**: Challenging integration or optimization (optional)
- **Difficulty Labels**: Explicit tags (Beginner/Intermediate/Advanced) set expectations
- **Solutions**: All required exercises + extensions have solutions in appendix

**Example (Chapter 2: ROS 2 Architecture):**
- **Required Exercise**: Build 2-node sensor fusion system (2 publishers, 1 subscriber) - Acceptance: Fusion node correctly combines sensor data
- **Extension - Beginner**: Add a 3rd sensor and visualize in rqt_graph
- **Extension - Intermediate**: Implement timestamped message synchronization
- **Extension - Advanced**: Add service-based reconfiguration for fusion weights

## Consequences

### Positive

- **Inclusive design**: Beginners not overwhelmed, can complete all chapters via required exercises only
- **Engaging for fast learners**: Advanced extensions provide challenge without penalty
- **Clear success path**: Required exercise defines minimum competency (graded), extensions are bonus
- **Flexible pacing**: Readers self-select difficulty, move at own pace
- **Reusable in courses**: Instructors can assign required exercises + select extensions by student level
- **Motivation**: Extensions provide growth path for readers who master basics quickly

### Negative

- **More content to write**: 3-4 exercises per chapter vs 1 (4x content creation)
- **Larger solution appendix**: Must provide solutions for required + all extensions (testing burden)
- **Difficulty calibration risk**: Mislabeling difficulty frustrates readers (requires beta testing)
- **Optional completion tracking**: Harder to measure reader success (did they do extensions?)
- **Maintenance burden**: More exercises = more code to maintain as tools evolve

## Alternatives Considered

**Alternative 1: Single required exercise only (no extensions)**
- Uniform experience, simpler to document
- Rejected: No challenge for fast learners, edge case "exercises scale for different learning speeds" unaddressed

**Alternative 2: Multiple required exercises (3-4 per chapter, all graded)**
- Deep practice for all readers
- Rejected: Overwhelming for beginners, increases time per chapter from 2-4 hours to 6-8 hours

**Alternative 3: Choose-your-own difficulty (pick 1 of 3 difficulty levels)**
- Customizable experience
- Rejected: Ambiguous success criteria (which level is "passing"?), harder to grade

**Alternative 4: Tiered chapters (Beginner chapters 1-8, Advanced chapters 9-16)**
- Clear difficulty progression
- Rejected: Doesn't address within-chapter learning speed variation, forces all readers through same pace

**Why Required + Optional Extensions Chosen:**
- Satisfies constitution: "one practical exercise with clear acceptance criteria" (FR-003)
- Addresses edge case: "Extension challenges for advanced learners" (spec.md Edge Cases)
- Inclusive: All readers can succeed (required only), fast learners stay engaged (extensions)
- Flexible: Readers self-regulate difficulty without author forcing pace

## References

- Feature Spec: [specs/001-physical-ai-book/spec.md](../../specs/001-physical-ai-book/spec.md) (FR-003: one exercise per chapter, Edge Cases: exercise scaling)
- Implementation Plan: [specs/001-physical-ai-book/plan.md](../../specs/001-physical-ai-book/plan.md) (lines 273-308)
- Related ADRs: ADR-003 (Content Depth Strategy)
- Constitution: Section III (Chapter structure), Section IV (Code examples must be tested)

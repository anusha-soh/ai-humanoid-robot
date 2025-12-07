# ADR-005: Content Progression Strategy

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-06
- **Feature:** 001-physical-ai-book
- **Context:** User guidance requested "Research-concurrent approach (iterate chapters with feedback)." Module dependencies exist (can't teach Nav2 without ROS 2 foundation). Need to balance speed with quality through reader feedback.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? YES - determines development workflow, timeline, quality
     2) Alternatives: Multiple viable options considered with tradeoffs? YES - Waterfall, parallel, iterative, various feedback strategies
     3) Scope: Cross-cutting concern (not an isolated detail)? YES - affects all 16 chapters, development timeline, beta reader management
-->

## Decision

Modules develop **sequentially** (Module 1 → 2 → 3 → 4 due to dependencies), but chapters within a module **iterate with reader feedback loops** before finalizing.

**Development Workflow:**

**Sequential Module Development:**
- Module 1 (Chapters 1-4) must complete before Module 2 starts (ROS 2 foundation required)
- Module 2 (Chapters 5-8) must complete before Module 3 starts (simulation foundation required)
- Module 3 (Chapters 9-12) must complete before Module 4 starts (perception foundation required)
- Module 4 (Chapters 13-16) integrates all previous modules (VLA capstone)

**Iterative Chapter Development (within each module):**
1. **Draft** (2-3 days): Write chapter following template, cite sources
2. **Code** (1 day): Implement and test all code examples
3. **Verify** (1 day): Run MCP server contacx7 on technical claims, update citations
4. **Review** (1 day): Share with 3-5 beta readers (AI practitioners matching target audience)
5. **Refine** (1-2 days): Address feedback, fix issues identified by beta readers
6. **Quality gates** (automated): Commit triggers readability, code, build checks (CI/CD)
7. **Finalize** (1 day): Technical accuracy review, plagiarism check

**Feedback Loop Strategy:**
- Draft chapter → Share with 3-5 beta readers → Collect feedback on clarity, exercise difficulty, code runability
- Refine based on feedback before finalizing
- Each module ends with integration test (readers test full module exercises sequentially)

**Timeline:**
- Phase 0: Setup (Week 1)
- Phase 1: Module 1 (Weeks 2-5, 1 week per chapter)
- Phase 2: Module 2 (Weeks 6-9)
- Phase 3: Module 3 (Weeks 10-13)
- Phase 4: Module 4 (Weeks 14-17)
- Phase 5: Polish (Weeks 18-20)

## Consequences

### Positive

- **Sequential modules prevent dependency confusion**: Readers can't access Module 3 without Module 1-2 foundation
- **Feedback loops improve quality**: Beta readers catch issues before wide release
- **Iterative refinement catches errors early**: Fix issues in Chapter 1 before patterns propagate to Chapter 16
- **Realistic timeline**: 5 months (20 weeks) with built-in feedback buffer
- **Reader-tested quality**: All chapters validated by target audience before publication
- **Constitution-compliant**: Allows MCP server verification per module (SC-018)

### Negative

- **Slower development**: Feedback adds 1-2 weeks per module (vs waterfall write-all-then-review)
- **Beta reader recruitment burden**: Need to maintain pool of 10-15 beta readers across 5 months
- **Blocking dependencies**: Can't parallelize Module 1 and Module 3 development (sequential constraint)
- **Feedback fatigue**: Beta readers may lose interest over 5 months (need cohort rotation strategy)
- **Rework risk**: Module 1 feedback may require changes to already-drafted later chapters

## Alternatives Considered

**Alternative 1: Waterfall (write all 16 chapters, then review all)**
- Faster initial writing
- Rejected: Errors propagate to all chapters, late feedback too expensive to fix, no early validation

**Alternative 2: Parallel development (all 4 modules simultaneously)**
- Fastest timeline (4 months vs 5 months)
- Rejected: Violates module dependencies (can't write Module 3 without Module 1 examples), inconsistent patterns across modules

**Alternative 3: No beta readers (author self-review only)**
- Simpler, faster (saves 1-2 weeks per module)
- Rejected: High risk of accessibility issues, missing reader perspective on difficulty calibration

**Alternative 4: Continuous feedback (publish chapters live, iterate based on reader comments)**
- Maximum feedback, very agile
- Rejected: Incomplete chapters confuse readers, hard to maintain quality gates on live content

**Why Sequential Modules + Iterative Chapters Chosen:**
- Honors user guidance: "Research-concurrent approach (iterate chapters with feedback)"
- Respects technical dependencies: Module 1 foundation enables Module 2-4
- Balances speed (iterative within module) and quality (feedback loops)
- Proven pattern: Similar to how technical books are developed (sequential modules, beta review per chapter)

## References

- Feature Spec: [specs/001-physical-ai-book/spec.md](../../specs/001-physical-ai-book/spec.md) (FR-004: progressive difficulty)
- Implementation Plan: [specs/001-physical-ai-book/plan.md](../../specs/001-physical-ai-book/plan.md) (lines 311-375, Progression Map section 322-358)
- Related ADRs: ADR-003 (Content Depth), ADR-004 (Exercise Structure)
- User Guidance: "Research-concurrent approach (iterate chapters with feedback)" (plan.md line 10, 316)
- Timeline: 20 weeks total (plan.md lines 322-358, 1114)

# ADR-001: Simulation Technology Stack

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-06
- **Feature:** 001-physical-ai-book
- **Context:** The book teaches Physical AI and Humanoid Robotics through simulation-first approach with zero hardware requirements. Readers range from beginners to advanced learners, with varying GPU access.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? YES - determines simulation APIs/SDKs throughout book
     2) Alternatives: Multiple viable options considered with tradeoffs? YES - Gazebo-only, Isaac-only, hybrid, multiple tools
     3) Scope: Cross-cutting concern (not an isolated detail)? YES - affects all modules, code examples, reader setup
-->

## Decision

Use **Gazebo Classic for Modules 1-2** (foundation, basic simulation), then transition to **NVIDIA Isaac Sim for Modules 3-4** (advanced perception, VLA).

**Technology Stack Components:**
- **Foundation Simulation (Modules 1-2)**: Gazebo Classic 11
- **Advanced Simulation (Modules 3-4)**: NVIDIA Isaac Sim 2023.1+
- **Fallback Strategy**: Provide Gazebo-only alternatives for Module 3-4 exercises for readers without GPU
- **Integration**: Both tools integrate with ROS 2 Humble via standard interfaces

## Consequences

### Positive

- **Progressive learning curve**: Gazebo in Modules 1-2 minimizes setup friction for beginners (lightweight, CPU-only)
- **Photorealistic perception**: Isaac Sim in Modules 3-4 provides GPU-accelerated rendering needed for computer vision and VLA systems
- **Accessible to all readers**: Gazebo fallback ensures readers without high-end GPUs can complete all exercises
- **Industry-relevant skills**: Readers learn both widely-used Gazebo and cutting-edge Isaac Sim platforms
- **Confidence building**: Readers master simpler Gazebo before tackling Isaac GPU requirements

### Negative

- **Dual installation burden**: Readers must install and learn two different simulation platforms
- **Code duplication**: Some exercises need dual implementations (Gazebo + Isaac variants)
- **Transition friction**: Module 2→3 transition requires learning new Isaac Sim APIs and workflows
- **Maintenance complexity**: Authors must maintain and test examples in both simulators
- **Documentation overhead**: Need platform-specific troubleshooting for two systems

## Alternatives Considered

**Alternative 1: Gazebo-only**
- Simpler setup, single platform to learn
- Rejected: Weak photorealistic rendering and perception capabilities insufficient for Module 3-4 computer vision and VLA content

**Alternative 2: Isaac Sim-only**
- Best quality, single platform
- Rejected: Heavy GPU requirements (RTX series) exclude readers without high-end hardware, creates accessibility barrier

**Alternative 3: Multiple simulation alternatives (Gazebo + Isaac + Unity + Webots)**
- Maximum flexibility
- Rejected: Too many tools create cognitive overload, excessive maintenance burden, dilutes expertise

**Why Hybrid Chosen:**
- Balances accessibility (Gazebo for beginners) with quality (Isaac for advanced AI/perception)
- Readers build confidence with Gazebo before tackling GPU-intensive Isaac
- Fallback strategy ensures inclusivity while teaching cutting-edge tools

## References

- Feature Spec: [specs/001-physical-ai-book/spec.md](../../specs/001-physical-ai-book/spec.md)
- Implementation Plan: [specs/001-physical-ai-book/plan.md](../../specs/001-physical-ai-book/plan.md) (lines 168-196)
- Related ADRs: None
- Success Criteria: SC-006 (all code examples run successfully), FR-009 (simulation-based, zero hardware)

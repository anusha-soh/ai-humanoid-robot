# ADR-002: LLM Integration Strategy

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-06
- **Feature:** 001-physical-ai-book
- **Context:** Module 4 (Vision-Language-Action) requires speech recognition and LLM task planning. Readers have varying preferences for privacy, cost, and complexity. Some prefer cloud APIs (simple), others prefer local models (privacy, offline).

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? YES - determines Module 4 architecture, API dependencies
     2) Alternatives: Multiple viable options considered with tradeoffs? YES - Cloud API vs local models vs hybrid
     3) Scope: Cross-cutting concern (not an isolated detail)? YES - affects Module 4 chapters 13-16, capstone project
-->

## Decision

**Primary path**: Use **OpenAI API** (Whisper for speech recognition, GPT-4 for task planning) with **documented local alternatives** (Whisper.cpp, Llama 2) for Module 4.

**Integration Strategy:**
- **Speech Recognition**: OpenAI Whisper API (primary), Whisper.cpp (local alternative documented in appendix)
- **Task Planning**: OpenAI GPT-4 API (primary), Llama 2/3 (local alternative documented in appendix)
- **Code Examples**: All Chapter 13-16 examples use OpenAI API with sidebar notes pointing to local alternatives
- **Appendix**: Full local model setup guide with equivalent code examples
- **Abstraction**: Use ROS 2 service interfaces to allow readers to swap backends easily

## Consequences

### Positive

- **Beginner-friendly**: OpenAI API setup simpler than downloading/configuring large local models
- **Free tier sufficient**: OpenAI free tier covers all book exercises (low cost for learning)
- **Flexibility**: Readers choose privacy (local) vs simplicity (API) based on preferences
- **Future-proof**: Abstraction layer allows swapping LLM backends without rewriting code
- **Consistent results**: API provides stable, predictable outputs for exercises
- **Fast iteration**: Readers can quickly test examples without model download wait times

### Negative

- **API key setup friction**: Requires OpenAI account creation and API key configuration
- **Internet dependency**: Primary path requires internet connectivity for API calls
- **Privacy concerns**: API sends data to OpenAI servers (mitigated by local alternative)
- **Documentation burden**: Must maintain two parallel paths (API + local) in appendix
- **Cost uncertainty**: API pricing may change, free tier limits may tighten
- **Vendor dependency**: Primary examples tied to OpenAI ecosystem

## Alternatives Considered

**Alternative 1: OpenAI API only (no local alternatives)**
- Simplest documentation
- Rejected: Excludes readers with privacy concerns or offline requirements, reduces flexibility

**Alternative 2: Local models only (Whisper.cpp + Llama 2)**
- Privacy-friendly, no API costs, offline-capable
- Rejected: Complex setup (download multi-GB models, configure inference), higher barrier for beginners

**Alternative 3: Cloud-only (multiple providers: OpenAI, Anthropic, Google)**
- Vendor flexibility
- Rejected: Too many setup paths confuse beginners, excessive documentation overhead

**Alternative 4: Hybrid with equal emphasis (50/50 API and local in main content)**
- Maximum flexibility
- Rejected: Dilutes focus, harder for beginners to follow, doubles code examples

**Why Flexible API Primary + Local Documented Chosen:**
- Beginner path (API) is clear and simple, advanced path (local) available for those who need it
- Abstraction via ROS 2 interfaces makes backend swapping straightforward
- Readers can start with API, migrate to local models later without rewriting code
- Balances accessibility (API simplicity) with privacy/offline needs (local documented)

## References

- Feature Spec: [specs/001-physical-ai-book/spec.md](../../specs/001-physical-ai-book/spec.md) (FR-008: Module 4 requirements)
- Implementation Plan: [specs/001-physical-ai-book/plan.md](../../specs/001-physical-ai-book/plan.md) (lines 199-233)
- Related ADRs: None
- Assumptions: "LLM API Access: readers can access OpenAI API or similar (free tier sufficient)" (spec.md line 278)

---
id: 003
title: Architectural Decisions Documentation
stage: misc
date: 2025-12-10
surface: agent
model: claude-opus-4-5-20251101
feature: rag-chatbot
branch: master
user: system
command: /sp.adr
labels: ["adr", "architecture", "decisions", "documentation"]
links:
  spec: specs/rag-chatbot/spec.md
  ticket: null
  adr: history/adr/011-backend-technology-stack.md,history/adr/012-llm-integration-strategy.md,history/adr/013-vector-database-architecture.md,history/adr/014-frontend-integration-approach.md,history/adr/015-streaming-protocol.md
  pr: null
files:
  - history/adr/011-backend-technology-stack.md
  - history/adr/012-llm-integration-strategy.md
  - history/adr/013-vector-database-architecture.md
  - history/adr/014-frontend-integration-approach.md
  - history/adr/015-streaming-protocol.md
tests:
  - None (documentation phase)
---

## Prompt

Review the generated plan and record key Architectural Decisions.

Focus on decisions that:
1. Affect multiple sections or phases
2. Had alternatives we considered

## Response snapshot

Analyzed specs/rag-chatbot/plan.md and extracted 5 architecturally significant decision clusters from the "Architectural Decisions" section (lines 7-59). Applied significance test to each cluster - all passed. Created 5 ADRs documenting decisions with rationale, alternatives, and consequences.

**Decision Clusters Identified:**

1. **Backend Technology Stack & Architecture** (ADR-011)
   - FastAPI with service-oriented layers vs monolithic
   - Affects all backend phases (1-6), testing strategy, scaling path

2. **LLM Integration Strategy** (ADR-012)
   - Gemini 2.0-flash-exp via OpenAI Agent SDK
   - Affects cost model, streaming, migration path
   - User-specified free tier requirement

3. **Vector Database & Embedding Architecture** (ADR-013)
   - Single Qdrant collection (768 dims, Cosine) vs multiple collections
   - Affects ingestion, retrieval, storage costs
   - User-clarified during /sp.clarify session

4. **Frontend Integration Approach** (ADR-014)
   - Docusaurus Root swizzling vs custom plugin
   - Affects upgrade path, maintenance, theming

5. **Real-Time Streaming Protocol** (ADR-015)
   - Server-Sent Events vs WebSockets
   - Affects user experience, latency, infrastructure

**Significance Test Results:**
- All 5 clusters passed 3-part test (Impact + Alternatives + Scope)
- No over-granular ADRs (all are clustered decisions)
- All include explicit alternatives with rejection rationale
- All cross-reference plan.md, spec.md, and related ADRs

**ADR Quality Checklist (ALL PASS):**
- ✅ Clustered decisions (not atomic choices)
- ✅ Explicit alternatives listed with pros/cons
- ✅ Consequences cover both positive and negative
- ✅ References link to plan + spec + related ADRs
- ✅ Significance checklist filled (Impact + Alternatives + Scope)

**No Conflicts Detected:**
- Existing ADRs (001-010) are for different feature (001-physical-ai-book)
- New ADRs (011-015) are first for rag-chatbot feature
- No overlapping decisions or contradictions

## Outcome

- ✅ Impact: 5 architecturally significant decisions documented with full context, alternatives, and tradeoffs. Team has permanent record of "why" for future reference.
- 🧪 Tests: None (documentation phase)
- 📁 Files: 5 created (ADR-011 through ADR-015)
- 🔁 Next prompts: Proceed to /sp.tasks for granular task breakdown; ADRs provide architectural foundation
- 🧠 Reflection: Plan's explicit "Architectural Decisions" section made extraction straightforward. All 5 decisions met significance criteria (no trivial choices). User clarifications from /sp.clarify (Gemini model, single collection) ensured ADRs reflect actual decisions, not hypotheticals.

## Evaluation notes (flywheel)

- Failure modes observed: None. All ADRs passed grader checklist (clustered, alternatives, consequences, references).
- Graders run and results (PASS/FAIL): PASS - All 5 ADRs have explicit alternatives, clustered decisions, clear consequences
- Prompt variant (if applicable): Standard /sp.adr workflow
- Next experiment (smallest change to try): During implementation (red/green), validate that ADRs correctly captured decisions (e.g., does SSE implementation match ADR-015 exactly?)

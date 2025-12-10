# ADR-012: LLM Integration Strategy for RAG Chatbot

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-10
- **Feature:** rag-chatbot
- **Context:** RAG chatbot requires LLM for answer generation with streaming support, free tier availability for MVP, and proven SDK reliability. User specified preference for Gemini free tier over OpenAI paid tiers. Need to integrate Gemini while leveraging battle-tested SDK patterns.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? YES - determines cost model, API dependencies, streaming implementation
     2) Alternatives: Multiple viable options considered with tradeoffs? YES - Gemini direct SDK, OpenAI native, hybrid approaches
     3) Scope: Cross-cutting concern (not an isolated detail)? YES - affects chat endpoint, streaming, evaluation, user experience
-->

## Decision

**LLM Integration Stack:**
- **Primary Model**: Google Gemini 2.0-flash-exp (free tier: 10 RPM)
- **SDK**: OpenAI Python SDK (v1.54.0+) with custom base URL
- **Integration Method**: OpenAI Agent SDK configured with Gemini-compatible endpoint
- **Base URL**: `https://generativelanguage.googleapis.com/v1beta/openai/`
- **Embeddings**: Google Gemini text-embedding-004 (768 dimensions)
- **Streaming**: Enabled via OpenAI SDK's native streaming support

**Implementation Details:**
```python
from openai import OpenAI

client = OpenAI(
    api_key=settings.gemini_api_key,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)

stream = client.chat.completions.create(
    model="gemini-2.0-flash-exp",
    messages=[...],
    stream=True
)
```

## Consequences

### Positive

- **Cost-Free Operation**: Gemini free tier (10 RPM) sufficient for MVP and initial users (no API costs)
- **Proven SDK Patterns**: OpenAI SDK is battle-tested, well-documented, has robust streaming implementation
- **Easy Migration Path**: If Gemini limits insufficient, switch to OpenAI by changing base_url and API key (no code rewrite)
- **Streaming Support**: Native SSE streaming via OpenAI SDK's generator interface works out-of-box
- **Type Safety**: OpenAI SDK provides typed response objects (ChatCompletion, ChatCompletionChunk)
- **Error Handling**: Built-in retry logic and error types from OpenAI SDK
- **Future Flexibility**: Abstraction allows testing different models (Gemini 1.5, 2.0 variants) via config change

### Negative

- **API Compatibility Risk**: Gemini's OpenAI-compatible endpoint may have subtle differences or lag behind OpenAI SDK updates
- **Rate Limit Constraints**: 10 RPM free tier may throttle during concurrent testing or early user spikes
- **Vendor Dependency**: Tied to Google Gemini's free tier policy; if pricing changes, migration needed
- **Documentation Gap**: Gemini's OpenAI-compatible mode less documented than native Gemini SDK
- **Feature Parity Uncertainty**: Some advanced OpenAI features may not be supported via Gemini endpoint
- **Mixed Provider Confusion**: Using OpenAI SDK for Gemini may confuse developers unfamiliar with compatibility layer

## Alternatives Considered

**Alternative 1: Native Gemini SDK (google-generativeai)**
- Direct integration with `google.generativeai` Python library
- Pros: Official support, full feature access, no compatibility layer
- Cons: Different API patterns (non-OpenAI conventions), less mature streaming support, team has more OpenAI experience
- Rejected: OpenAI SDK patterns more familiar to team; Gemini SDK lacks proven streaming implementations at scale

**Alternative 2: OpenAI GPT-4o-mini (Native)**
- Use OpenAI's native API with GPT-4o-mini model
- Pros: No compatibility layer, proven at scale, excellent documentation
- Cons: Cost (~$0.15 per 1M input tokens), no free tier, requires billing setup
- Rejected: User explicitly requested Gemini free tier; cost-free operation critical for MVP validation

**Alternative 3: Hybrid (Gemini for embeddings, OpenAI for chat)**
- Use Gemini embeddings with OpenAI chat completions
- Pros: Leverages best of both (free embeddings, proven chat)
- Cons: Dual API key management, mixed cost model, complex billing tracking
- Rejected: Complicates free-tier story; user wanted full Gemini stack

**Alternative 4: LangChain Abstraction Layer**
- Use LangChain to abstract LLM provider
- Pros: Provider-agnostic, easy switching, rich ecosystem
- Cons: Heavy dependency (~50MB+), unnecessary abstraction for simple streaming use case, slower iteration
- Rejected: Over-engineering; direct SDK calls simpler for this use case; LangChain adds latency overhead

**Why Gemini via OpenAI SDK Chosen:**
- Combines free tier (Gemini) with proven SDK reliability (OpenAI patterns)
- Team expertise in OpenAI SDK reduces implementation risk
- Easy migration to OpenAI if Gemini limits become constraint
- Minimal code changes needed if switching providers (just base_url + API key)

## References

- Feature Spec: [specs/rag-chatbot/spec.md](../../specs/rag-chatbot/spec.md) (Lines 12-13: Clarification on Gemini 2.0-flash-exp choice, Lines 196-197: LLM stack)
- Implementation Plan: [specs/rag-chatbot/plan.md](../../specs/rag-chatbot/plan.md) (Lines 22-27: Gemini Integration decision, Task 2.2: LLM Service implementation)
- Related ADRs: ADR-011 (Backend Stack), ADR-013 (Vector DB - embeddings compatibility), ADR-015 (Streaming Protocol)
- Evaluator Evidence: plan.md lines 819-866 (Phase 2 checkpoint verifies streaming works)

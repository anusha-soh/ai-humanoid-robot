# ADR-015: Real-Time Streaming Communication Protocol

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-10
- **Feature:** rag-chatbot
- **Context:** RAG chatbot must stream LLM responses token-by-token for low perceived latency, maintain connection during multi-second generation, and degrade gracefully on network failures. Target p95 latency of < 1s for first token, < 10s for complete response. Must work across browsers (Chrome, Firefox, Safari) without requiring WebSocket infrastructure.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? YES - determines user experience (latency perception), backend streaming implementation, infrastructure needs
     2) Alternatives: Multiple viable options considered with tradeoffs? YES - SSE vs WebSockets vs polling vs single response
     3) Scope: Cross-cutting concern (not an isolated detail)? YES - affects chat endpoint (Phase 2), frontend client (Phase 3), deployment (load balancer config)
-->

## Decision

**Streaming Architecture:**
- **Protocol**: Server-Sent Events (SSE) via HTTP/1.1 persistent connection
- **Backend**: FastAPI `StreamingResponse` with async generator
- **Frontend**: Native `fetch()` API with ReadableStream reader
- **Message Format**: Newline-delimited JSON (`data: {...}\n\n`)
- **Event Types**:
  - `{"type": "token", "content": "word"}` - Incremental token
  - `{"type": "done", "sources": [...]}` - Stream completion with citations
- **Connection Handling**: Auto-reconnect on disconnect (frontend retry logic)
- **Fallback**: Degrade to single-response mode if SSE fails after 3 retries

**Backend Implementation (FastAPI):**
```python
async def event_stream():
    async for token in llm_service.stream_completion(...):
        response = ChatResponse(type="token", content=token)
        yield f"data: {response.model_dump_json()}\n\n"

    response = ChatResponse(type="done", sources=sources)
    yield f"data: {response.model_dump_json()}\n\n"

return StreamingResponse(
    event_stream(),
    media_type="text/event-stream",
    headers={"Cache-Control": "no-cache", "Connection": "keep-alive"}
)
```

**Frontend Implementation (React):**
```typescript
const reader = response.body?.getReader();
const decoder = new TextDecoder();

while (true) {
    const { done, value } = await reader.read();
    if (done) break;

    const text = decoder.decode(value, { stream: true });
    // Parse "data: {...}\n\n" format
}
```

## Consequences

### Positive

- **Native Browser Support**: SSE works in all modern browsers without libraries (no socket.io dependency)
- **Simple Backend**: FastAPI `StreamingResponse` + async generator = clean implementation
- **Low Latency**: Tokens arrive as generated (sub-100ms incremental updates)
- **Automatic Reconnection**: Browsers auto-reconnect on disconnect (resilient to network hiccups)
- **Firewall-Friendly**: Uses standard HTTP/HTTPS (no blocked ports like WebSockets often are)
- **Load Balancer Compatible**: Works through nginx/cloudflare without special WebSocket config
- **Type-Safe Events**: JSON format allows Pydantic validation on backend, TypeScript types on frontend

### Negative

- **Unidirectional Only**: Client can't send data over SSE connection (must use separate POST for new messages)
- **HTTP/1.1 Limitation**: Browsers limit concurrent SSE connections per domain (~6); not issue for single chat widget
- **Text-Only Protocol**: Can't send binary data (not needed for text chat)
- **Connection Overhead**: Persistent HTTP connection held open during streaming (resource held per user)
- **Buffering Issues**: Some proxies/CDNs buffer SSE; must configure `X-Accel-Buffering: no`
- **No Built-in Heartbeat**: Must manually implement keepalive if needed (e.g., send comments every 30s)

## Alternatives Considered

**Alternative 1: WebSockets (Full Bidirectional)**
- Use `websockets` library (Python) + WebSocket API (browser)
- Pros: True bidirectional, can push updates to client, lower overhead once established
- Cons: More complex (handshake, ping/pong, connection management), requires WebSocket-aware load balancer, firewall issues, overkill for unidirectional streaming
- Rejected: SSE simpler for one-way streaming; bidirectional not needed (client sends POST for new messages)

**Alternative 2: HTTP Long Polling**
- Client repeatedly requests, server holds connection until data ready
- Pros: Works everywhere (oldest browsers, strict firewalls)
- Cons: Higher latency (request overhead per chunk), complex server state management, inefficient (constant reconnections)
- Rejected: SSE native reconnection better; long polling outdated pattern

**Alternative 3: HTTP/2 Server Push**
- Use HTTP/2 push frames to send tokens
- Pros: Multiplexed streams, lower overhead
- Cons: Deprecated in Chrome/Firefox (removed 2022), complex server setup, not well-supported
- Rejected: Browser support withdrawn; SSE more future-proof

**Alternative 4: Single Response (No Streaming)**
- Wait for full LLM completion, return entire response at once
- Pros: Simplest implementation, no connection management, works everywhere
- Cons: Poor UX (10s wait with no feedback), violates SC-001 (< 1s first token), feels unresponsive
- Rejected: Streaming essential for perceived performance; users need immediate feedback

**Alternative 5: gRPC Streaming**
- Use gRPC with server streaming RPC
- Pros: Efficient binary protocol, strong typing, HTTP/2 multiplexing
- Cons: Requires gRPC-web proxy for browsers, complex setup, heavier tooling, poor debugging
- Rejected: Over-engineering; SSE sufficient for text streaming; gRPC adds deployment complexity

**Why Server-Sent Events Chosen:**
- Native browser support (no libraries needed)
- Simple backend (async generator pattern)
- Firewall/load-balancer friendly (standard HTTP)
- Sufficient for unidirectional LLM streaming
- Proven pattern for similar use cases (ChatGPT, Anthropic Claude use SSE)
- Easy debugging (human-readable text format, visible in browser DevTools)

## References

- Feature Spec: [specs/rag-chatbot/spec.md](../../specs/rag-chatbot/spec.md) (FR-007: Must stream responses, SC-001: < 1s first token latency)
- Implementation Plan: [specs/rag-chatbot/plan.md](../../specs/rag-chatbot/plan.md) (Lines 53-58: Streaming Architecture decision, Task 2.4: Chat endpoint with SSE, Task 3.2: Frontend SSE client)
- Related ADRs: ADR-012 (LLM Integration - streaming source), ADR-014 (Frontend Integration - client implementation)
- MDN Web Docs: https://developer.mozilla.org/en-US/docs/Web/API/Server-sent_events (SSE specification)
- Evaluator Evidence: plan.md lines 819-866 (Phase 2 checkpoint verifies streaming works, Phase 3 checkpoint verifies frontend consumes stream)

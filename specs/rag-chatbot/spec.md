# Feature Specification: RAG-Powered Chatbot for Docusaurus

**Feature Branch**: `003-rag-chatbot`
**Created**: 2025-12-10
**Status**: Draft
**Input**: User description: "Create a RAG chatbot system integrated with my existing Docusaurus book project with OpenAI, Qdrant, FastAPI, and text selection mode"

## Clarifications

### Session 2025-12-10
- Q: Qdrant collection strategy - single or multiple collections? → A: Single collection with metadata filtering
- Q: LLM model selection - GPT-4o vs GPT-4o-mini? → A: Gemini gemini-2.0-flash-exp via OpenAI Agent SDK (OpenAI-compatible interface)
- Q: Embedding provider - OpenAI or Gemini? → A: Gemini text-embedding-004 (free tier, 768 dimensions)
- Q: CORS allowed origins - which URLs? → A: http://localhost:3000 (dev) and https://ai-humanoid-robot.vercel.app (prod)
- Q: Qdrant vector dimensions and similarity metric? → A: 768 dimensions with Cosine similarity

## User Scenarios & Testing

### User Story 1 - Basic Chat Interaction (Priority: P1)

A reader visits the Docusaurus book and wants to ask questions about the content. They open the chat widget, type their question, and receive an AI-generated answer based on the book's content with relevant source citations.

**Why this priority**: This is the core value proposition - enabling readers to interact with the book content conversationally. Without this, no other features matter.

**Independent Test**: Can be fully tested by opening the chat widget, asking "What is physical AI?", and verifying that the response references relevant book content with proper citations. Delivers immediate value by making book content queryable.

**Acceptance Scenarios**:

1. **Given** a reader is on any Docusaurus page, **When** they click the chat widget icon, **Then** the chat interface opens with a welcome message
2. **Given** the chat is open, **When** the user types "What are the main components of a humanoid robot?" and submits, **Then** the system retrieves relevant chunks, generates an answer using Gemini, and displays the response with source references
3. **Given** a chat response is displayed, **When** the user clicks a source citation, **Then** they are navigated to the specific section of the book
4. **Given** the user has asked multiple questions, **When** they scroll up, **Then** they can see the full conversation history in the current session

---

### User Story 2 - Text Selection Context Mode (Priority: P2)

A reader is studying a specific paragraph about robot control systems. They highlight the text with their mouse, and the chat widget automatically switches to "selection mode" where all questions are answered based ONLY on the selected text, not the entire book.

**Why this priority**: This differentiates the chatbot from generic Q&A - it enables deep, focused exploration of specific concepts. Essential for learning workflows.

**Independent Test**: Can be tested by highlighting a paragraph about "sensor fusion", asking "Explain this concept in simpler terms", and verifying the answer references ONLY the selected text. Works independently of general chat.

**Acceptance Scenarios**:

1. **Given** a user is reading a page, **When** they highlight text (mouseup event), **Then** the chat widget shows a "Selection Mode Active" indicator with the selected text preview
2. **Given** selection mode is active, **When** the user asks "What does this mean?", **Then** the response is generated using ONLY the selected text as context, not the full book
3. **Given** selection mode is active, **When** the user clicks "Exit Selection Mode", **Then** the chat returns to normal RAG mode across all content
4. **Given** the user makes a new text selection, **When** they already had text selected, **Then** the context switches to the new selection

---

### User Story 3 - Document Ingestion and Indexing (Priority: P1)

An admin needs to index all Docusaurus markdown files into the vector database. They run the ingestion script which parses all markdown, chunks the content intelligently (preserving headings and context), generates embeddings, and stores them in Qdrant.

**Why this priority**: Without indexed content, the chatbot cannot function. This is infrastructure for P1 and P2.

**Independent Test**: Can be tested by running the ingestion command, verifying all markdown files are processed, and querying Qdrant to confirm vectors are stored with proper metadata. Delivers the knowledge base required for chat.

**Acceptance Scenarios**:

1. **Given** Docusaurus markdown files exist in `/docs`, **When** the admin runs `python backend/ingest.py`, **Then** all markdown files are discovered and parsed
2. **Given** markdown files are parsed, **When** content is chunked, **Then** each chunk preserves heading hierarchy, stays under 512 tokens, and includes metadata (file path, heading, section)
3. **Given** chunks are created, **When** embeddings are generated, **Then** Gemini API is called with `text-embedding-004` model for each chunk
4. **Given** embeddings are ready, **When** they are stored, **Then** Qdrant collection `docusaurus-book` contains vectors with metadata (chunk_id, file_path, heading, content, token_count)
5. **Given** ingestion completes, **When** the admin queries Qdrant, **Then** the collection count matches the expected number of chunks

---

### User Story 4 - Conversation Persistence (Priority: P3)

A user has a conversation with the chatbot across multiple questions. When they close the browser and return later, they want to optionally resume their previous conversation or start fresh.

**Why this priority**: Enhances user experience but not critical for MVP. Users can still get value from ephemeral sessions.

**Independent Test**: Can be tested by having a conversation, closing the browser, reopening, and verifying the conversation is retrievable. Works independently by storing/loading from Postgres.

**Acceptance Scenarios**:

1. **Given** a user starts a new chat session, **When** they send their first message, **Then** a new `conversation_id` is generated and stored in Neon Postgres
2. **Given** a conversation is active, **When** each message is sent/received, **Then** it is stored in the `messages` table with (conversation_id, role, content, timestamp)
3. **Given** the user returns later, **When** they open the chat widget, **Then** they see an option to "Resume Last Conversation" or "Start New"
4. **Given** the user resumes, **When** the chat loads, **Then** previous messages are fetched from Postgres and displayed in order

---

### User Story 5 - Health Monitoring and Error Handling (Priority: P3)

The system administrator needs to monitor the health of the chatbot services. The FastAPI backend exposes health check endpoints that verify connectivity to Qdrant and Neon Postgres.

**Why this priority**: Important for production reliability but not needed for initial development and testing.

**Independent Test**: Can be tested by calling `/health` endpoint and verifying it returns status of all dependencies. Works independently without chat functionality.

**Acceptance Scenarios**:

1. **Given** the FastAPI backend is running, **When** a client calls `GET /health`, **Then** the response includes status of: FastAPI (healthy), Qdrant (connected/disconnected), Postgres (connected/disconnected), Gemini (API key valid/invalid)
2. **Given** Qdrant is unreachable, **When** `/health` is called, **Then** the response indicates Qdrant is down with error details
3. **Given** all services are healthy, **When** `/health` is called, **Then** the response returns 200 OK with detailed status JSON

---

### User Story 6 - RAG Quality Evaluation (Priority: P1)

The development team needs to verify that the RAG pipeline retrieves relevant content accurately. They create a golden evaluation dataset with 20 domain-relevant questions and 5 out-of-scope questions, each with expected chunk IDs or similarity thresholds.

**Why this priority**: Without evaluation, we cannot verify SC-004 (90% accuracy at >0.7 similarity). This is foundational for measuring RAG quality.

**Independent Test**: Can be tested by running the evaluation script against the golden dataset and verifying accuracy metrics. Works independently once ingestion is complete.

**Acceptance Scenarios**:

1. **Given** evaluation dataset with 20 domain questions and expected chunk IDs, **When** RAG pipeline processes each query, **Then** at least 90% retrieve expected chunks in top-5 results with similarity > 0.7
2. **Given** 5 out-of-scope questions (e.g., "What is the weather?", "Who won the 2024 election?"), **When** processed, **Then** all return similarity scores < 0.5
3. **Given** out-of-scope questions with low similarity, **When** chatbot generates response, **Then** it returns "I couldn't find relevant information in the book about that" without hallucinating
4. **Given** evaluation script runs, **When** it completes, **Then** it outputs metrics: precision@5, recall@5, average similarity score, and pass/fail status

---

### Edge Cases

- **What happens when user highlights text across multiple paragraphs?** System should capture the full selection up to a maximum token limit (e.g., 2000 tokens) and truncate with a warning if exceeded.
- **What happens when Gemini API rate limit is hit?** System should return a graceful error message: "Service temporarily busy, please try again in a moment" and log the error.
- **What happens when no relevant chunks are found in Qdrant?** System should respond: "I couldn't find relevant information in the book about that. Could you rephrase or ask something else?"
- **What happens when user asks a question unrelated to the book content?** System should detect low similarity scores and respond: "This question seems outside the scope of this book. I can only answer questions based on the book content."
- **What happens when markdown parsing fails for a file?** Ingestion script should log the error, skip the file, and continue processing other files. Final report should list failed files.
- **What happens when Qdrant Cloud free tier storage limit is reached?** System should fail gracefully during ingestion with clear error message indicating storage quota exceeded.

## Requirements

### Functional Requirements

- **FR-001**: System MUST provide a chat widget UI component that can be embedded in Docusaurus pages
- **FR-002**: System MUST detect text selection events (mouseup) on Docusaurus pages and enable "selection mode"
- **FR-003**: System MUST distinguish between general chat mode (full book RAG) and selection mode (selected text only) in the UI
- **FR-004**: Backend MUST expose a `/chat` endpoint accepting: `message: string`, `mode: "general" | "selection"`, `context?: string` (selected text if mode=selection), `conversation_id?: string`
- **FR-005**: System MUST perform semantic search in Qdrant to retrieve top-k (k=5) most relevant chunks based on query embedding
- **FR-006**: System MUST construct a prompt for Gemini including: system instructions, retrieved chunks (or selected text), and user question
- **FR-007**: System MUST stream Gemini responses back to the frontend for real-time display
- **FR-008**: System MUST include source citations in responses, linking back to original document sections
- **FR-009**: System MUST store conversation history in Neon Postgres with schema: `conversations(id, created_at)` and `messages(id, conversation_id, role, content, timestamp)`
- **FR-010**: Ingestion pipeline MUST parse all markdown files from Docusaurus `/docs` directory
- **FR-011**: Ingestion pipeline MUST chunk content using semantic boundaries:
  - Primary boundaries: H2 and H3 markdown headings
  - Preserve parent heading hierarchy as prefix (e.g., "Hardware > Sensors > Text content")
  - Max 512 tokens per chunk; split long sections at paragraph boundaries
  - Include metadata: file_path, heading_path (list), section_depth (int)
- **FR-012**: Ingestion pipeline MUST generate embeddings using Google Gemini `text-embedding-004` model (768 dimensions)
- **FR-013**: Ingestion pipeline MUST store embeddings in Qdrant Cloud collection `docusaurus-book` (768 dimensions, Cosine similarity) with metadata: chunk_id, file_path, heading, content, token_count
- **FR-014**: Backend MUST expose `/health` endpoint returning status of all dependencies
- **FR-015**: System MUST validate Gemini API key on startup and fail fast with clear error if invalid
- **FR-016**: System MUST handle Qdrant connection failures gracefully and return appropriate error messages
- **FR-017**: System MUST be deployable via Docker with docker-compose orchestrating frontend, backend, and supporting services
- **FR-018**: Backend services MUST have pytest integration tests covering all API endpoints (/chat, /health)
- **FR-019**: Frontend components MUST have Jest unit tests with React Testing Library (ChatWidget, MessageList, InputBox, SelectionMode)
- **FR-020**: Test coverage MUST meet: backend services ≥80%, frontend components ≥70%
- **FR-021**: Ingestion pipeline MUST have test fixtures with sample markdown files (valid, malformed, empty) to verify parsing logic
- **FR-022**: Chat widget MUST support keyboard navigation (Tab to focus, Enter to submit, Escape to close widget)
- **FR-023**: All interactive elements MUST have ARIA labels for screen reader compatibility (button roles, live regions for messages)
- **FR-024**: Chat widget MUST respect `prefers-reduced-motion` user preference for animations
- **FR-025**: Conversation context (last 3 user+assistant message pairs) applies to general mode only for follow-up question coherence
- **FR-026**: Selection mode MUST NOT include conversation history; responses generated using ONLY current selected text as context
- **FR-027**: Integration MUST NOT break Docusaurus build process (`npm run build` must succeed with zero errors)

### Key Entities

- **Conversation**: Represents a chat session. Attributes: id (UUID), created_at (timestamp). Stored in Postgres.
- **Message**: Represents a single message in a conversation. Attributes: id (UUID), conversation_id (FK), role (user|assistant|system), content (text), timestamp. Stored in Postgres.
- **Chunk**: Represents a segment of book content. Attributes: chunk_id (UUID), file_path (string), heading (string), content (text), token_count (int), embedding (vector). Stored in Qdrant.
- **Document**: Represents a markdown file from Docusaurus. Attributes: file_path (string), raw_content (text), headings (list), metadata (dict). Transient during ingestion.

## Success Criteria

### Measurable Outcomes

- **SC-001**: Users can ask a question and receive a relevant answer with source citations in under 5 seconds (p95 latency)
- **SC-002**: Selection mode correctly constrains answers to ONLY the highlighted text, verified by test cases where general knowledge would give different answers
- **SC-003**: Ingestion pipeline successfully processes 100% of valid markdown files without crashes (invalid files logged and skipped)
- **SC-004**: RAG retrieval returns relevant chunks with similarity score > 0.7 for 90% of domain-relevant questions
- **SC-005**: Chat widget integrates into Docusaurus with zero layout breaks or z-index conflicts on all standard screen sizes (desktop, tablet, mobile)
- **SC-006**: System handles 50 concurrent users without response time degradation beyond 10%
- **SC-007**: Health check endpoint responds within 500ms and accurately reflects dependency status
- **SC-008**: Docker deployment completes successfully with `docker-compose up` with no manual intervention required
- **SC-009**: All test suites pass in CI/CD pipeline: pytest (backend), Jest (frontend), integration tests complete in <60 seconds
- **SC-010**: Chat widget passes axe DevTools accessibility audit with zero violations (WCAG 2.1 AA compliance)
- **SC-011**: Docusaurus build succeeds with chat widget integration (`npm run build` produces zero errors)
- **SC-012**: Evaluation dataset tests pass with ≥90% accuracy (18/20 domain questions retrieve expected chunks at >0.7 similarity)

## Technical Constraints

### Stack and Dependencies

- **Frontend**: React 18+, TypeScript, must integrate with Docusaurus 3.x theming system
- **Backend**: FastAPI (Python 3.11+), async/await patterns for streaming responses
- **LLM**: Google Gemini (gemini-2.0-flash-exp) via OpenAI Agent SDK with OpenAI-compatible API interface, streaming enabled
- **Embeddings**: Google Gemini `text-embedding-004` (768 dimensions) via Gemini API free tier
- **Vector DB**: Qdrant Cloud free tier (1GB storage limit, ~300k vectors with metadata)
- **Relational DB**: Neon Serverless Postgres (free tier: 10GB storage, 100 hours compute/month)
- **Deployment**: Docker and docker-compose for local development and production

### Non-Functional Requirements

- **Performance**: Chat responses must stream within 1s of first token, complete within 10s for p95
- **Scalability**: Architecture must support horizontal scaling of FastAPI backend (stateless design)
- **Security**:
  - API keys stored in `.env` files, never committed to git
  - CORS configured to allow: `http://localhost:3000` (development) and `https://ai-humanoid-robot.vercel.app` (production)
  - No user authentication required (public read-only chatbot)
- **Reliability**:
  - Retry logic for Gemini API calls (3 retries with exponential backoff)
  - Graceful degradation if Qdrant is down (error message, no crash)
  - Database connection pooling for Postgres
- **Observability**:
  - Structured logging (JSON format) for all API requests
  - Log levels: DEBUG for development, INFO for production
  - Error tracking includes: timestamp, request_id, user message, error type, stack trace

### Integration Points

- **Docusaurus Integration**: Chat widget injected via custom Docusaurus plugin or theme swizzling
- **Gemini API**: RESTful API calls via OpenAI Agent SDK (OpenAI-compatible interface), streaming responses
- **Qdrant Cloud**: gRPC or HTTP API, connection string via environment variable
- **Neon Postgres**: Connection via `asyncpg` or `psycopg3`, connection pooling enabled
- **Docker Network**: Frontend (port 3000), Backend (port 8000), internal network for service discovery

## Architecture Overview

### Component Structure

```
/backend
  /app
    /api
      - chat.py          # /chat endpoint
      - health.py        # /health endpoint
    /services
      - rag_service.py   # RAG pipeline orchestration
      - embedding.py     # Gemini embedding generation
      - retrieval.py     # Qdrant vector search
      - llm.py           # Gemini chat completion with streaming (via OpenAI SDK)
    /models
      - schemas.py       # Pydantic models for requests/responses
    /db
      - postgres.py      # Neon Postgres connection and queries
      - qdrant.py        # Qdrant client and operations
    - main.py            # FastAPI app initialization
  /scripts
    - ingest.py          # Document ingestion pipeline
    - evaluate.py        # RAG evaluation script with golden dataset
  /tests
    /integration
      - test_chat_api.py # pytest for /chat endpoint
      - test_health_api.py # pytest for /health endpoint
    /unit
      - test_rag_service.py
      - test_embedding.py
    /fixtures
      - sample_markdown/  # Test markdown files
      - golden_dataset.json # Evaluation questions and expected chunks
  - Dockerfile
  - requirements.txt
  - .env.example
  - pytest.ini
  - .coveragerc

/frontend
  /src
    /components
      - ChatWidget.tsx   # Main chat UI component
      - MessageList.tsx  # Conversation display
      - InputBox.tsx     # User input
      - SelectionMode.tsx # Selection mode indicator
    /hooks
      - useChat.ts       # Chat logic and API calls
      - useTextSelection.ts # Selection detection
    /api
      - client.ts        # Backend API client
    /__tests__
      - ChatWidget.test.tsx
      - MessageList.test.tsx
      - useChat.test.ts
      - useTextSelection.test.ts
  - package.json
  - tsconfig.json
  - jest.config.js

/docs                     # Existing Docusaurus content
docker-compose.yml
```

### Data Flows

**General Chat Flow**:
1. User types message in ChatWidget
2. Frontend calls `POST /chat` with `{message, mode: "general", conversation_id?}`
3. Backend generates embedding for user message via Gemini
4. Backend queries Qdrant for top-5 similar chunks
5. Backend constructs prompt: system instructions + retrieved chunks + user question
6. Backend calls Gemini streaming completion (via OpenAI SDK)
7. Backend streams response tokens back to frontend via Server-Sent Events (SSE)
8. Backend stores user message and assistant response in Postgres
9. Frontend displays response with source citations

**Selection Mode Flow**:
1. User highlights text on page (mouseup event)
2. Frontend captures selected text via `window.getSelection()`
3. Frontend displays "Selection Mode Active" badge
4. User asks question
5. Frontend calls `POST /chat` with `{message, mode: "selection", context: selectedText}`
6. Backend uses ONLY `context` field (no Qdrant query)
7. Backend constructs prompt: system instructions + selected text + user question
8. Backend calls Gemini streaming completion (via OpenAI SDK)
9. Backend streams response back to frontend

**Ingestion Flow**:
1. Admin runs `python scripts/ingest.py`
2. Script discovers all `.md` files in `/docs` directory
3. For each file:
   a. Parse markdown and extract structured content
   b. Chunk content (max 512 tokens, preserve heading context)
   c. Generate embedding via Gemini `text-embedding-004`
   d. Upsert to Qdrant with metadata
4. Script logs progress and final statistics

### API Contracts

**POST /chat**
```json
Request:
{
  "message": "What is sensor fusion?",
  "mode": "general" | "selection",
  "context": "optional selected text if mode=selection",
  "conversation_id": "optional UUID"
}

Response (SSE stream):
data: {"type": "token", "content": "Sensor"}
data: {"type": "token", "content": " fusion"}
data: {"type": "done", "sources": [{"file": "/docs/hardware/sensors.md", "heading": "Sensor Fusion"}]}
```

**GET /health**
```json
Response:
{
  "status": "healthy",
  "services": {
    "fastapi": "ok",
    "qdrant": {"status": "connected", "collection": "docusaurus-book", "vectors": 15234},
    "postgres": {"status": "connected"},
    "gemini": {"status": "ok", "model": "gemini-2.0-flash-exp"}
  },
  "timestamp": "2025-12-10T12:34:56Z"
}
```

## Out of Scope

- User authentication and authorization (chatbot is public and read-only)
- Multi-language support (English only for MVP)
- Voice input/output
- Real-time collaborative chat (multiple users in same conversation)
- Fine-tuning custom embeddings or LLM models
- Advanced analytics dashboard
- Integration with external knowledge bases beyond Docusaurus content
- Mobile native apps (web-only via responsive design)

## Open Questions

~~1. **Chunking Strategy**: Should we use fixed-size chunks (512 tokens) or semantic chunking based on paragraph/section boundaries?~~
   - **RESOLVED**: FR-011 specifies semantic chunking with H2/H3 boundaries, parent heading preservation

~~2. **Citation Formatting**: How should we display source citations - inline links, footnotes, or sidebar?~~
   - **RESOLVED**: Inline links within response text like `[source: Hardware Overview]` that navigate to the section (FR-008)

~~3. **Conversation Context**: Should follow-up questions include previous messages for context?~~
   - **RESOLVED**: FR-025/FR-026 specify: general mode includes last 3 pairs, selection mode excludes history

~~4. **Qdrant Collection Strategy**: Single collection for all content or separate collections per book section?~~
   - **RESOLVED**: Single collection `docusaurus-book` with metadata filtering (file_path, module) for all content; enables future extensibility without complexity

~~5. **OpenAI Model Selection**: GPT-4o (more accurate) vs GPT-4o-mini (faster, cheaper)?~~
   - **RESOLVED**: Use Google Gemini model via OpenAI Agent SDK (OpenAI-compatible API interface); provides cost efficiency with Gemini's generous free tier

## Dependencies and Risks

### External Dependencies

- **Qdrant Cloud**: Free tier limits (1GB storage, no SLA). Risk: Quota exceeded or service downtime.
  - *Mitigation*: Monitor usage, implement local Qdrant fallback with Docker for development
- **Neon Postgres**: Free tier limits (100 hours compute/month). Risk: Quota exceeded mid-month.
  - *Mitigation*: Monitor usage, consider local Postgres for development
- **Gemini API**: Rate limits on free tier (10 RPM for gemini-2.0-flash-exp). Risk: Throttling during concurrent usage.
  - *Mitigation*: Implement rate limiting on backend, queue requests during peak load, consider caching frequent queries

### Technical Risks

- **Docusaurus Integration Complexity**: Custom plugin may break on Docusaurus version upgrades
  - *Mitigation*: Use stable Docusaurus APIs, comprehensive integration tests
- **Embedding Quality**: Text chunking may split related content, reducing retrieval quality
  - *Mitigation*: Experiment with chunk sizes (256, 512, 1024 tokens) and overlap strategies
- **Streaming Reliability**: SSE connections may timeout or fail on slow networks
  - *Mitigation*: Implement reconnection logic, fallback to non-streaming responses

## Next Steps

1. **Planning Phase**: Run `/sp.plan` to architect detailed implementation plan
2. **Task Breakdown**: Run `/sp.tasks` to generate actionable task list with dependencies
3. **Environment Setup**: Configure `.env` files for Gemini API, Qdrant Cloud, and Neon Postgres credentials
4. **Development Priority**: Implement in order: P1 stories (Basic Chat + Ingestion), P2 (Selection Mode), P3 (Persistence + Health)

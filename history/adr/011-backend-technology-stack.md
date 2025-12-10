# ADR-011: Backend Technology Stack & Architecture

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-10
- **Feature:** rag-chatbot
- **Context:** RAG chatbot backend needs to orchestrate multiple operations (embedding generation, vector search, LLM streaming, conversation persistence) with testability, maintainability, and future horizontal scaling requirements. Initial implementation targets single-server deployment with potential for future distributed architecture.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? YES - defines backend structure for all 6 phases
     2) Alternatives: Multiple viable options considered with tradeoffs? YES - layered vs monolithic
     3) Scope: Cross-cutting concern (not an isolated detail)? YES - affects API, services, testing, deployment
-->

## Decision

**Backend Stack:**
- **Framework**: FastAPI (Python 3.11+)
- **Architecture**: Service-oriented layered architecture
- **Async Pattern**: async/await throughout for I/O operations
- **Dependency Injection**: Singleton service instances
- **Layer Structure**:
  - **API Layer** (`/app/api`): FastAPI routers for endpoints
  - **Service Layer** (`/app/services`): Business logic (RAG pipeline, embedding, LLM)
  - **Data Layer** (`/app/db`): Database clients (Qdrant, Postgres)
  - **Models Layer** (`/app/models`): Pydantic schemas

**Key Services:**
- `rag_service.py`: Orchestrates RAG pipeline (embedding → retrieval → generation)
- `embedding.py`: Handles Gemini text-embedding-004 API calls with retry logic
- `llm.py`: Gemini chat completion with streaming via OpenAI Agent SDK
- `retrieval.py`: Qdrant vector search operations

## Consequences

### Positive

- **Testability**: Service layer can be unit tested independently of API layer (mock external calls)
- **Maintainability**: Clear separation of concerns makes code easier to navigate and modify
- **Scalability**: Stateless design enables horizontal scaling (multiple FastAPI instances behind load balancer)
- **Type Safety**: Pydantic models provide runtime validation and IDE autocomplete
- **Async Performance**: Non-blocking I/O enables handling multiple concurrent requests efficiently
- **Future-Proof**: Layered architecture allows swapping implementations (e.g., different vector DB) without API changes

### Negative

- **Initial Boilerplate**: More files and directory structure than single-file app (~15 files vs 1)
- **Learning Curve**: Developers need to understand layer boundaries and dependency flow
- **Over-engineering Risk**: Layered architecture may be overkill if feature never scales beyond single use case
- **Import Complexity**: More modules means more import statements and potential circular dependency issues
- **Testing Overhead**: Need to test both unit (service layer) and integration (full stack) paths

## Alternatives Considered

**Alternative 1: Monolithic Single-File App**
- All code in `main.py` (~500-800 lines)
- Pros: Simple, fast to prototype, fewer files
- Cons: Hard to test (tight coupling), difficult to maintain as features grow, no clear separation of concerns
- Rejected: Technical debt accumulates quickly; harder to onboard new developers; testing requires full app context

**Alternative 2: Microservices Architecture**
- Separate services for embedding, retrieval, LLM, persistence
- Pros: Maximum decoupling, independent deployment, language flexibility
- Cons: Network overhead, complex orchestration, operational complexity, overkill for MVP
- Rejected: Premature optimization; adds deployment/monitoring complexity without current need

**Alternative 3: Modular Monolith (Django-style)**
- Django REST Framework with apps for each domain
- Pros: Batteries-included (ORM, admin, auth), mature ecosystem
- Cons: Heavier framework, less control over async patterns, opinionated structure
- Rejected: FastAPI's async-first design better suits streaming use case; lighter weight for API-focused backend

**Why Service-Oriented FastAPI Chosen:**
- Balances maintainability (layered) with simplicity (single deployment unit)
- FastAPI's async support essential for streaming LLM responses without blocking
- Pydantic integration provides strong typing and validation without boilerplate
- Clear migration path to microservices if needed (extract services → deploy separately)

## References

- Feature Spec: [specs/rag-chatbot/spec.md](../../specs/rag-chatbot/spec.md) (FR-004 to FR-016: Backend requirements)
- Implementation Plan: [specs/rag-chatbot/plan.md](../../specs/rag-chatbot/plan.md) (Lines 8-20: Backend Architecture section, Phase 1-2)
- Related ADRs: ADR-012 (LLM Integration), ADR-013 (Vector DB Architecture)
- Evaluator Evidence: plan.md lines 482-523 (Phase 1 checkpoint verifies structure)

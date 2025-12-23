# Implementation Plan: Hugging Face Spaces Backend Deployment

**Branch**: `003-hf-spaces-deploy` | **Date**: 2025-12-22 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/003-hf-spaces-deploy/spec.md`

## Summary

Deploy the existing FastAPI RAG chatbot backend to Hugging Face Spaces using Docker SDK. The deployment requires creating HF Spaces-compatible Dockerfile (port 7860, non-root user with UID 1000), README.md with YAML frontmatter, and documentation for configuring environment secrets. The frontend on Vercel will be updated to point to the new HF Spaces backend URL.

## Technical Context

**Language/Version**: Python 3.11 (existing backend)
**Primary Dependencies**: FastAPI 0.115.0, uvicorn 0.32.0, qdrant-client 1.12.0, google-generativeai 0.8.0, sqlalchemy 2.0.35, psycopg 3.2.3
**Storage**: PostgreSQL (Neon - external), Qdrant Cloud (external)
**Testing**: pytest with pytest-asyncio (existing test suite)
**Target Platform**: Hugging Face Spaces (Docker SDK, free tier)
**Project Type**: Web application (FastAPI backend + React frontend on Vercel)
**Performance Goals**: Health endpoint < 5s, first token < 10s (including cold start)
**Constraints**: Port 7860 (HF requirement), non-root user UID 1000 (HF security), free tier resource limits
**Scale/Scope**: Single Space deployment, existing external services (Neon PostgreSQL, Qdrant Cloud, Gemini API)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Accuracy and Source Transparency | PASS | HF Spaces documentation referenced |
| II. Consistency Across Chapters | N/A | Not content creation |
| III. Modular Structure | PASS | Deployment files in separate directory |
| IV. Code Quality and Testing | PASS | Dockerfile will be tested via HF build |
| V. Accessibility and Clarity | PASS | Step-by-step documentation provided |
| VI. Deployment and Build Standards | PASS | Docker build must succeed on HF |
| VII. Originality and Attribution | PASS | No external code required |

**Constitution Compliance**: All applicable principles satisfied. This is a deployment task, not content creation, so book-specific principles (II, III) are N/A.

## Project Structure

### Documentation (this feature)

```text
specs/003-hf-spaces-deploy/
├── plan.md              # This file
├── research.md          # Phase 0 - HF Spaces requirements research
├── data-model.md        # Phase 1 - Deployment artifacts model
├── quickstart.md        # Phase 1 - Quick deployment guide
├── contracts/           # Phase 1 - N/A (no new API contracts)
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
# Existing backend structure (unchanged)
backend/
├── app/
│   ├── api/            # Existing endpoints (health.py, chat.py)
│   ├── core/           # Configuration (config.py)
│   ├── db/             # Database clients (postgres.py, qdrant.py)
│   ├── models/         # Pydantic schemas
│   ├── services/       # RAG service, embedding service
│   └── main.py         # FastAPI application
├── Dockerfile          # Existing Docker config (port 8000)
└── requirements.txt    # Python dependencies

# NEW: HF Spaces deployment files
huggingface-spaces/
├── README.md           # HF Spaces YAML frontmatter + description
├── Dockerfile          # HF-specific Docker config (port 7860, non-root)
├── requirements.txt    # Copy of backend requirements
└── app/                # Symlink or copy of backend/app
```

**Structure Decision**: Create a separate `huggingface-spaces/` directory containing HF-specific deployment files to avoid conflicts with the existing `backend/Dockerfile`. This preserves the original setup for local development while providing HF-specific configuration.

## Complexity Tracking

> No Constitution violations. All requirements align with minimal complexity.

| Aspect | Approach | Justification |
|--------|----------|---------------|
| Separate deployment directory | `huggingface-spaces/` | Avoids modifying existing backend for HF-specific requirements (port, user) |
| File duplication | Copy app code, not symlink | HF Spaces uploads files, doesn't support symlinks |

## Phase 0: Research Summary

### HF Spaces Docker Requirements (Verified)

**Source**: [Hugging Face Spaces Docker Documentation](https://huggingface.co/docs/hub/en/spaces-sdks-docker)

1. **Port Configuration**:
   - Default port: 7860
   - Set via `app_port: 7860` in README.md YAML
   - Can have internal ports (e.g., for local Elasticsearch) but only one exposed

2. **Non-Root User Requirement**:
   - Must create user with UID 1000: `RUN useradd -m -u 1000 user`
   - Switch to user: `USER user`
   - Set home: `ENV HOME=/home/user`
   - Use `--chown=user` for all COPY/ADD commands

3. **README.md YAML Frontmatter**:
   ```yaml
   ---
   title: RAG Chatbot Backend
   emoji: 🤖
   colorFrom: blue
   colorTo: purple
   sdk: docker
   app_port: 7860
   ---
   ```

4. **Secrets Management**:
   - Runtime: Secrets available as environment variables (e.g., `os.environ.get("SECRET_NAME")`)
   - Buildtime: Mount secrets with `RUN --mount=type=secret,id=SECRET_NAME`
   - Our use case: Runtime only (API keys, DB URLs)

5. **Startup Timeout**:
   - Default: 30 minutes
   - Can customize with `startup_duration_timeout: 5m`

### Dockerfile Pattern for FastAPI on HF Spaces

```dockerfile
FROM python:3.11-slim

# Install system dependencies
RUN apt-get update && apt-get install -y \
    gcc g++ curl \
    && rm -rf /var/lib/apt/lists/*

# Create non-root user (required by HF Spaces)
RUN useradd -m -u 1000 user
USER user

ENV HOME=/home/user \
    PATH=/home/user/.local/bin:$PATH

WORKDIR $HOME/app

# Install Python dependencies
COPY --chown=user requirements.txt .
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY --chown=user . .

# Health check for HF monitoring
HEALTHCHECK --interval=30s --timeout=10s --start-period=60s --retries=3 \
    CMD curl -f http://localhost:7860/api/health || exit 1

# Expose port 7860 (HF Spaces requirement)
EXPOSE 7860

# Run on port 7860
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "7860"]
```

### Required Environment Secrets

| Secret Name | Description | Source |
|-------------|-------------|--------|
| `GEMINI_API_KEY` | Google Gemini API key | https://aistudio.google.com/ |
| `QDRANT_URL` | Qdrant Cloud cluster URL | https://qdrant.tech/ |
| `QDRANT_API_KEY` | Qdrant Cloud API key | https://qdrant.tech/ |
| `QDRANT_COLLECTION` | Collection name | `docusaurus-book` |
| `DATABASE_URL` | Neon PostgreSQL connection string | https://neon.tech/ |
| `ALLOWED_ORIGINS` | Frontend URLs (comma-separated) | e.g., `https://your-app.vercel.app` |
| `LOG_LEVEL` | Logging verbosity | `INFO` |

### Frontend Configuration

The frontend uses `REACT_APP_API_BASE_URL` environment variable:
- Current: Falls back to `http://localhost:8000` in development
- Production on HF Spaces: `https://username-spacename.hf.space`

Update in Vercel dashboard: `REACT_APP_API_BASE_URL=https://username-rag-chatbot.hf.space`

## Phase 1: Design Artifacts

### Deployment Files Structure

```text
huggingface-spaces/
├── README.md           # HF YAML frontmatter + Space description
├── Dockerfile          # HF-specific (port 7860, user 1000)
├── requirements.txt    # Identical to backend/requirements.txt
├── app/
│   ├── __init__.py
│   ├── main.py         # FastAPI app entry point
│   ├── api/
│   │   ├── __init__.py
│   │   ├── chat.py
│   │   └── health.py
│   ├── core/
│   │   ├── __init__.py
│   │   └── config.py
│   ├── db/
│   │   ├── __init__.py
│   │   ├── postgres.py
│   │   └── qdrant.py
│   ├── models/
│   │   ├── __init__.py
│   │   └── schemas.py
│   └── services/
│       ├── __init__.py
│       ├── embedding.py
│       └── rag_service.py
└── scripts/
    └── chunker.py      # If needed for runtime
```

### Data Model: Deployment Artifacts

| Artifact | Purpose | Location |
|----------|---------|----------|
| HF Dockerfile | HF-specific Docker config | `huggingface-spaces/Dockerfile` |
| HF README.md | Space configuration + description | `huggingface-spaces/README.md` |
| requirements.txt | Python dependencies | `huggingface-spaces/requirements.txt` |
| App code | Backend application | `huggingface-spaces/app/` |
| Deployment guide | Step-by-step instructions | `specs/003-hf-spaces-deploy/quickstart.md` |

### Contracts

No new API contracts required. The deployment preserves all existing endpoints:
- `GET /api/health` - Health check (unchanged)
- `POST /api/chat` - Chat with SSE streaming (unchanged)
- `GET /api/conversation/{id}` - Retrieve conversation (unchanged)

## Implementation Phases

### Phase 1: Create HF Spaces Directory Structure
1. Create `huggingface-spaces/` directory
2. Create HF-specific `Dockerfile` with:
   - Non-root user (UID 1000)
   - Port 7860 exposure
   - HEALTHCHECK instruction
3. Create `README.md` with YAML frontmatter
4. Copy `requirements.txt` from backend
5. Copy application code from `backend/app/`

### Phase 2: Create Deployment Documentation
1. Write `quickstart.md` with step-by-step deployment guide
2. Document secret configuration in HF Settings
3. Document frontend URL update process
4. Include verification steps

### Phase 3: Frontend Configuration Update
1. Document how to update `REACT_APP_API_BASE_URL` in Vercel
2. Ensure CORS is configured for HF Spaces URL format

### Phase 4: Testing & Verification
1. Deploy to HF Spaces
2. Verify health endpoint
3. Test chat functionality
4. Verify frontend connectivity

## Risk Analysis

| Risk | Impact | Mitigation |
|------|--------|------------|
| HF Space sleep after inactivity | High latency on first request | Document expected behavior; health endpoint returns "starting" during wake |
| Build failures on HF | Deployment blocked | Test Dockerfile locally first |
| Secret misconfiguration | Backend non-functional | Clear documentation + health endpoint shows connection status |

## Open Questions

None. All requirements are well-defined by the specification and HF documentation.

---

**Next Step**: Run `/sp.tasks` to generate the implementation task list.

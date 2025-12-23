# Research: Hugging Face Spaces Deployment

**Feature**: 003-hf-spaces-deploy
**Date**: 2025-12-22
**Status**: Complete

## Research Topics

### 1. HF Spaces Docker SDK Requirements

**Decision**: Use Docker SDK with port 7860 and non-root user UID 1000

**Rationale**:
- HF Spaces requires Docker containers to expose port 7860 (configurable via `app_port`)
- Security policy mandates non-root user with UID 1000
- Docker SDK is the only option for custom backend applications (Gradio/Streamlit are for ML demos)

**Alternatives Considered**:
- Gradio SDK: Not suitable for FastAPI backends
- Streamlit SDK: Not suitable for API-only backends
- Static SDK: No backend capability

**Sources**:
- [HF Spaces Docker Documentation](https://huggingface.co/docs/hub/en/spaces-sdks-docker)
- [HF Spaces Configuration Reference](https://huggingface.co/docs/hub/en/spaces-config-reference)

---

### 2. Non-Root User Configuration

**Decision**: Create user with `useradd -m -u 1000 user` and use `--chown=user` for all file operations

**Rationale**:
- HF Spaces runs containers with UID 1000
- Using a different UID causes permission issues
- All files must be owned by the user to avoid write permission errors

**Implementation Pattern**:
```dockerfile
# Create non-root user (required by HF Spaces)
RUN useradd -m -u 1000 user
USER user
ENV HOME=/home/user PATH=/home/user/.local/bin:$PATH
WORKDIR $HOME/app

# Always use --chown=user
COPY --chown=user requirements.txt .
COPY --chown=user . .
```

**Alternatives Considered**:
- Running as root: Violates HF Spaces security policy
- Using different UID: Causes permission issues

**Sources**:
- [HF Spaces Docker Permissions](https://huggingface.co/docs/hub/en/spaces-sdks-docker#permissions)

---

### 3. Secrets Management

**Decision**: Use HF Space Settings for runtime environment variables

**Rationale**:
- Secrets set in HF Settings are available at runtime as environment variables
- No code changes needed - existing `os.environ.get()` calls work as-is
- Secrets are encrypted at rest in HF infrastructure

**Required Secrets**:
| Secret | Description |
|--------|-------------|
| `GEMINI_API_KEY` | Google Gemini API key |
| `QDRANT_URL` | Qdrant Cloud cluster URL |
| `QDRANT_API_KEY` | Qdrant Cloud API key |
| `QDRANT_COLLECTION` | Collection name (docusaurus-book) |
| `DATABASE_URL` | Neon PostgreSQL connection string |
| `ALLOWED_ORIGINS` | Vercel frontend URLs |
| `LOG_LEVEL` | Logging level (INFO) |

**Alternatives Considered**:
- Build-time secrets: More complex, requires `RUN --mount=type=secret`
- Hardcoded values: Security violation
- Config files: Would be exposed in Docker image

**Sources**:
- [HF Spaces Secrets Documentation](https://huggingface.co/docs/hub/en/spaces-sdks-docker#secrets)

---

### 4. Health Check Configuration

**Decision**: Add HEALTHCHECK instruction to Dockerfile for HF monitoring

**Rationale**:
- HF Spaces monitors container health
- Existing `/api/health` endpoint returns HTTP 200/503
- HEALTHCHECK ensures HF can detect and restart unhealthy containers

**Implementation**:
```dockerfile
HEALTHCHECK --interval=30s --timeout=10s --start-period=60s --retries=3 \
    CMD curl -f http://localhost:7860/api/health || exit 1
```

**Parameters**:
- `interval=30s`: Check every 30 seconds
- `timeout=10s`: Fail if no response in 10s
- `start-period=60s`: Allow 60s for initial startup
- `retries=3`: Mark unhealthy after 3 consecutive failures

**Alternatives Considered**:
- No HEALTHCHECK: HF Spaces may not detect unhealthy state
- Shorter interval: Unnecessary overhead
- Longer start-period: Delays detection of real issues

---

### 5. Deployment Directory Strategy

**Decision**: Create separate `huggingface-spaces/` directory with copied files

**Rationale**:
- Preserves existing `backend/Dockerfile` for local development
- HF Spaces requires files in root of uploaded content
- Avoids port conflicts (8000 for local, 7860 for HF)
- Clear separation of concerns

**Implementation**:
```
huggingface-spaces/
├── README.md       # HF YAML frontmatter
├── Dockerfile      # HF-specific config
├── requirements.txt
└── app/            # Copied from backend/app
```

**Alternatives Considered**:
- Modify existing backend/Dockerfile: Would break local dev workflow
- Symlinks: Not supported by HF Spaces upload
- Docker multi-stage with different entrypoints: Overcomplicated

---

### 6. Frontend CORS Configuration

**Decision**: Update ALLOWED_ORIGINS to include HF Spaces URL pattern

**Rationale**:
- Frontend on Vercel makes cross-origin requests to HF Spaces backend
- CORS must allow the Vercel domain
- HF Spaces URL format: `https://username-spacename.hf.space`

**Implementation**:
```
ALLOWED_ORIGINS=https://your-app.vercel.app,https://username-rag-chatbot.hf.space
```

**Note**: Include the HF Spaces URL itself to allow Swagger UI testing from the Space.

---

### 7. Cold Start Behavior

**Decision**: Document expected cold start latency and design for it

**Rationale**:
- Free tier HF Spaces go to sleep after ~15 minutes of inactivity
- Wake-up time can be 30-60 seconds
- Health endpoint returns "starting" status during initialization

**Handling**:
1. Frontend shows loading indicator during cold start
2. Health endpoint returns 503 with "starting" status until ready
3. Once warm, responses are normal latency

**User Experience**:
- First request after sleep: 30-60 seconds
- Subsequent requests: Normal latency (<5s for health, <10s for first chat token)

---

## Summary

All research topics resolved. No "NEEDS CLARIFICATION" items remaining.

| Topic | Status | Decision |
|-------|--------|----------|
| Docker SDK requirements | Resolved | Port 7860, Docker SDK |
| Non-root user | Resolved | UID 1000, --chown=user |
| Secrets management | Resolved | HF Settings runtime variables |
| Health check | Resolved | HEALTHCHECK with curl |
| Directory strategy | Resolved | Separate huggingface-spaces/ |
| CORS configuration | Resolved | Include Vercel + HF URLs |
| Cold start | Resolved | Document expected behavior |

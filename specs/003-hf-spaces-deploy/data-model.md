# Data Model: HF Spaces Deployment Artifacts

**Feature**: 003-hf-spaces-deploy
**Date**: 2025-12-22

## Overview

This document describes the deployment artifacts required for Hugging Face Spaces deployment. No new database entities or API contracts are introduced - this deployment reuses the existing backend application.

## Deployment Artifacts

### 1. HF Spaces README.md

**Purpose**: Configure the HF Space with required YAML frontmatter

**Structure**:
```yaml
---
title: RAG Chatbot Backend
emoji: 🤖
colorFrom: blue
colorTo: purple
sdk: docker
app_port: 7860
short_description: FastAPI backend for RAG chatbot with Gemini + Qdrant
---
```

**Fields**:
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| title | string | Yes | Display name in HF interface |
| emoji | string | Yes | Single emoji for Space icon |
| colorFrom | string | Yes | Gradient start (red/yellow/green/blue/indigo/purple/pink/gray) |
| colorTo | string | Yes | Gradient end |
| sdk | string | Yes | Must be "docker" |
| app_port | int | Yes | Must be 7860 |
| short_description | string | No | Thumbnail description |

---

### 2. HF Spaces Dockerfile

**Purpose**: Build and run the FastAPI application on HF infrastructure

**Key Requirements**:
| Requirement | Value | Reason |
|-------------|-------|--------|
| Base image | python:3.11-slim | Match existing backend |
| User UID | 1000 | HF Spaces security requirement |
| Exposed port | 7860 | HF Spaces default |
| HEALTHCHECK | curl /api/health | Container health monitoring |

**Structure**:
```dockerfile
FROM python:3.11-slim

# System dependencies
RUN apt-get update && apt-get install -y gcc g++ curl && rm -rf /var/lib/apt/lists/*

# Non-root user
RUN useradd -m -u 1000 user
USER user
ENV HOME=/home/user PATH=/home/user/.local/bin:$PATH
WORKDIR $HOME/app

# Python dependencies
COPY --chown=user requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Application code
COPY --chown=user . .

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=60s --retries=3 \
    CMD curl -f http://localhost:7860/api/health || exit 1

EXPOSE 7860
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "7860"]
```

---

### 3. Environment Secrets

**Purpose**: Secure configuration for external service connections

**Required Secrets**:
| Name | Type | Example | Source |
|------|------|---------|--------|
| GEMINI_API_KEY | string | AIza... | Google AI Studio |
| QDRANT_URL | URL | https://xxx.qdrant.io | Qdrant Cloud |
| QDRANT_API_KEY | string | xxx | Qdrant Cloud |
| QDRANT_COLLECTION | string | docusaurus-book | Existing collection |
| DATABASE_URL | URL | postgresql://... | Neon |
| ALLOWED_ORIGINS | string | https://app.vercel.app | Vercel |
| LOG_LEVEL | string | INFO | - |

**Configuration Location**: HF Space Settings > Repository Secrets

---

### 4. Application Code Structure

**Purpose**: FastAPI backend application (copied from backend/app/)

**Directory Structure**:
```
app/
├── __init__.py
├── main.py              # FastAPI app entry point
├── api/
│   ├── __init__.py
│   ├── chat.py          # POST /api/chat (SSE streaming)
│   └── health.py        # GET /api/health
├── core/
│   ├── __init__.py
│   └── config.py        # Pydantic settings
├── db/
│   ├── __init__.py
│   ├── postgres.py      # PostgreSQL client
│   └── qdrant.py        # Qdrant client
├── models/
│   ├── __init__.py
│   └── schemas.py       # Pydantic models
└── services/
    ├── __init__.py
    ├── embedding.py     # Gemini embeddings
    └── rag_service.py   # RAG pipeline
```

**No Changes Required**: The application code is deployed as-is. Port configuration is handled by uvicorn command-line argument.

---

## Existing Entities (Unchanged)

These entities exist in external services and are not modified by this deployment:

### PostgreSQL Tables (Neon)

| Table | Purpose |
|-------|---------|
| conversations | Chat session metadata |
| messages | Individual chat messages |

### Qdrant Collections (Qdrant Cloud)

| Collection | Purpose |
|------------|---------|
| docusaurus-book | Document embeddings for RAG |

---

## Validation Rules

### README.md Validation
- [ ] YAML frontmatter starts with `---` and ends with `---`
- [ ] `sdk` is exactly "docker"
- [ ] `app_port` is exactly 7860
- [ ] `title` is non-empty string
- [ ] `emoji` is single emoji character

### Dockerfile Validation
- [ ] Uses `python:3.11-slim` or compatible base
- [ ] Creates user with UID 1000
- [ ] Uses `USER user` before app code copy
- [ ] All COPY commands use `--chown=user`
- [ ] EXPOSE is 7860
- [ ] CMD runs uvicorn on port 7860

### Secrets Validation
- [ ] All required secrets are set in HF Settings
- [ ] DATABASE_URL uses `sslmode=require` for Neon
- [ ] ALLOWED_ORIGINS includes Vercel frontend URL
- [ ] No secrets are hardcoded in code or Dockerfile

# Hugging Face Spaces Deployment Guide

A complete guide for deploying FastAPI RAG chatbot backends to Hugging Face Spaces.

## Table of Contents

1. [Overview](#overview)
2. [Prerequisites](#prerequisites)
3. [Quick Start](#quick-start)
4. [Detailed Steps](#detailed-steps)
5. [Configuration Reference](#configuration-reference)
6. [Troubleshooting](#troubleshooting)
7. [Lessons Learned](#lessons-learned)

---

## Overview

This guide covers deploying a FastAPI-based RAG (Retrieval-Augmented Generation) chatbot to Hugging Face Spaces using the Docker SDK. The architecture:

```
┌─────────────────┐     ┌─────────────────────────┐
│  Vercel Frontend │────▶│  HF Spaces Backend      │
│  (Docusaurus)    │     │  (FastAPI + Docker)     │
└─────────────────┘     └───────────┬─────────────┘
                                    │
                    ┌───────────────┼───────────────┐
                    ▼               ▼               ▼
              ┌──────────┐   ┌──────────┐   ┌──────────┐
              │  Qdrant  │   │   Neon   │   │  Gemini  │
              │  Cloud   │   │ Postgres │   │   API    │
              └──────────┘   └──────────┘   └──────────┘
```

---

## Prerequisites

### External Services (must be configured before deployment)

| Service | Purpose | Setup URL |
|---------|---------|-----------|
| Hugging Face | Backend hosting | [huggingface.co](https://huggingface.co) |
| Qdrant Cloud | Vector database | [cloud.qdrant.io](https://cloud.qdrant.io) |
| Neon | PostgreSQL database | [neon.tech](https://neon.tech) |
| Google AI Studio | Gemini API | [aistudio.google.com](https://aistudio.google.com) |
| Vercel | Frontend hosting | [vercel.com](https://vercel.com) |

### Local Requirements

- Git
- Node.js (for frontend)
- HF Access Token (for git push)

---

## Quick Start

```bash
# 1. Create HF Space at huggingface.co/new-space (select Docker SDK)

# 2. Clone and copy files
git clone https://huggingface.co/spaces/YOUR_USERNAME/rag-chatbot
cp -r huggingface-spaces/* rag-chatbot/
cd rag-chatbot

# 3. Push to HF
git add . && git commit -m "Deploy" && git push

# 4. Configure secrets in HF Space Settings

# 5. Verify
curl https://YOUR_USERNAME-rag-chatbot.hf.space/api/health
```

---

## Detailed Steps

### Step 1: Create HF Space

1. Go to [huggingface.co/new-space](https://huggingface.co/new-space)
2. Enter Space name: `rag-chatbot`
3. Select **Docker** as the SDK
4. Choose visibility (Public/Private)
5. Click **Create Space**

Your Space URL: `https://USERNAME-rag-chatbot.hf.space`

### Step 2: Prepare Deployment Files

#### Directory Structure

```
huggingface-spaces/
├── Dockerfile
├── README.md
├── requirements.txt
└── app/
    ├── __init__.py
    ├── main.py
    ├── api/
    │   ├── __init__.py
    │   ├── chat.py
    │   └── health.py
    ├── core/
    │   ├── __init__.py
    │   └── config.py
    ├── db/
    │   ├── __init__.py
    │   ├── postgres.py
    │   └── qdrant.py
    ├── models/
    │   ├── __init__.py
    │   └── schemas.py
    └── services/
        ├── __init__.py
        ├── embedding.py
        ├── llm.py
        └── rag_service.py
```

#### Dockerfile (Critical Requirements)

```dockerfile
FROM python:3.11-slim

# System dependencies
RUN apt-get update && apt-get install -y gcc g++ curl && rm -rf /var/lib/apt/lists/*

# Non-root user (REQUIRED: UID must be 1000)
RUN useradd -m -u 1000 user
USER user
ENV HOME=/home/user PATH=/home/user/.local/bin:$PATH
WORKDIR $HOME/app

# Python dependencies (REQUIRED: --chown=user)
COPY --chown=user requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Application code (REQUIRED: --chown=user)
COPY --chown=user . .

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=60s --retries=3 \
    CMD curl -f http://localhost:7860/api/health || exit 1

# REQUIRED: Port must be 7860
EXPOSE 7860
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "7860"]
```

#### README.md (YAML Frontmatter)

```yaml
---
title: RAG Chatbot Backend
emoji: "\U0001F916"
colorFrom: blue
colorTo: purple
sdk: docker
app_port: 7860
short_description: FastAPI RAG chatbot with Gemini + Qdrant
---
```

### Step 3: Configure Secrets

In HF Space **Settings** > **Repository Secrets** > **New secret**:

| Secret Name | Value | Notes |
|-------------|-------|-------|
| `GEMINI_API_KEY` | `AIza...` | From Google AI Studio |
| `QDRANT_URL` | `https://xxx.qdrant.io` | **No trailing space!** |
| `QDRANT_API_KEY` | `eyJ...` | From Qdrant Cloud |
| `QDRANT_COLLECTION` | `docusaurus-book` | Your collection name |
| `DATABASE_URL` | `postgresql://...?sslmode=require` | **Must include sslmode** |
| `ALLOWED_ORIGINS` | `https://your-app.vercel.app` | Frontend URL |
| `LOG_LEVEL` | `INFO` | |

### Step 4: Push to HF Spaces

```bash
# Clone HF Space repo
git clone https://huggingface.co/spaces/USERNAME/rag-chatbot hf-deploy

# Copy deployment files
cp -r huggingface-spaces/* hf-deploy/
cd hf-deploy

# Commit and push
git add .
git commit -m "Deploy RAG chatbot backend"
git push
```

**Authentication:** HF requires access token, not password
- Get token: [huggingface.co/settings/tokens](https://huggingface.co/settings/tokens)
- Use token as password when git prompts

### Step 5: Update Frontend

**CRITICAL: Do NOT use `process.env` in browser code!**

Wrong (causes "process is not defined" error):
```typescript
// DON'T DO THIS
this.baseUrl = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000';
```

Correct (use hostname detection):
```typescript
constructor() {
  const isProduction = typeof window !== 'undefined' &&
    !window.location.hostname.includes('localhost');

  this.baseUrl = isProduction
    ? 'https://USERNAME-rag-chatbot.hf.space'
    : 'http://localhost:8000';
}
```

### Step 6: Verify Deployment

```bash
# Check health
curl https://USERNAME-rag-chatbot.hf.space/api/health

# Expected response
{
  "status": "healthy",
  "services": {
    "fastapi": "ok",
    "qdrant": {"status": "connected", "collection": "docusaurus-book", "vectors": 562},
    "postgres": {"status": "connected"},
    "gemini": {"status": "ok", "model": "gemini-2.0-flash-exp"}
  }
}
```

---

## Configuration Reference

### Environment Variables

| Variable | Required | Default | Description |
|----------|----------|---------|-------------|
| `GEMINI_API_KEY` | Yes | - | Google Gemini API key |
| `GEMINI_MODEL` | No | `gemini-2.0-flash-exp` | LLM model |
| `GEMINI_EMBEDDING_MODEL` | No | `text-embedding-004` | Embedding model |
| `QDRANT_URL` | Yes | - | Qdrant Cloud URL |
| `QDRANT_API_KEY` | Yes | - | Qdrant API key |
| `QDRANT_COLLECTION` | No | `docusaurus-book` | Collection name |
| `DATABASE_URL` | Yes | - | PostgreSQL connection string |
| `ALLOWED_ORIGINS` | Yes | `http://localhost:3000` | CORS origins |
| `LOG_LEVEL` | No | `INFO` | Logging level |

### API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Root - returns API info |
| `/docs` | GET | Swagger UI |
| `/api/health` | GET | Health check (200/503) |
| `/api/chat` | POST | Chat with SSE streaming |
| `/api/conversation/{id}` | GET | Get conversation history |

### Health Status Codes

| Status | HTTP Code | Meaning |
|--------|-----------|---------|
| `healthy` | 200 | All services connected |
| `degraded` | 503 | One service down |
| `unhealthy` | 503 | Multiple services down |
| `starting` | 503 | App still initializing |

---

## Troubleshooting

### Qdrant Returns 404

**Symptom:** Health shows `qdrant: disconnected` with 404 error

**Causes:**
1. Qdrant cluster suspended (free tier sleeps after ~15min inactivity)
2. Wrong URL (check for typos or extra spaces)
3. Collection doesn't exist

**Fix:**
1. Go to [cloud.qdrant.io](https://cloud.qdrant.io)
2. Check cluster status - click **Resume** if suspended
3. Verify collection exists

### CORS Errors

**Symptom:** Browser console shows "blocked by CORS policy"

**Fix:**
1. Go to HF Space Settings > Repository Secrets
2. Update `ALLOWED_ORIGINS` to include your frontend URL:
   ```
   https://your-app.vercel.app,https://username-rag-chatbot.hf.space
   ```
3. Restart Space (Settings > Factory reboot)

### "process is not defined" Error

**Symptom:** Blank page, console shows `ReferenceError: process is not defined`

**Cause:** Using `process.env` in browser code (Docusaurus doesn't polyfill it)

**Fix:** Replace with hostname detection:
```typescript
const isProduction = typeof window !== 'undefined' &&
  !window.location.hostname.includes('localhost');
```

### Build Fails

**Common causes:**
- Missing `--chown=user` on COPY commands
- Wrong Python version
- Missing system dependencies (gcc, g++)

### Cold Start Slow

**Expected:** Free tier HF Spaces sleep after ~15min, wake-up takes 30-60 seconds

**During startup:** `/api/health` returns 503 with `status: "starting"`

---

## Lessons Learned

### From This Deployment Session

1. **Secret values must not have leading/trailing spaces** - Copy-paste from env files often includes extra spaces that break connections

2. **Don't use `process.env` in browser code** - Use `window.location.hostname` to detect environment instead

3. **Qdrant free tier suspends clusters** - Always check cluster status when seeing 404 errors

4. **HF Spaces requires access tokens for git** - Password authentication is deprecated; use tokens from huggingface.co/settings/tokens

5. **Port must be 7860** - HF Spaces expects this port; other ports won't work

6. **UID must be 1000** - HF Spaces runs containers as user 1000; permission errors occur otherwise

7. **Test on preview before merging** - But remember preview URLs aren't in ALLOWED_ORIGINS

8. **Include HF Spaces URL in CORS** - Allows testing from Swagger UI at `/docs`

---

## Using the Skill

To deploy another RAG chatbot, run:

```
/deploy-hf-spaces
```

Or with arguments:

```
/deploy-hf-spaces username=my-username space=my-chatbot frontend=https://my-app.vercel.app
```

---

## Resources

- [HF Spaces Docker SDK Docs](https://huggingface.co/docs/hub/en/spaces-sdks-docker)
- [HF Spaces Configuration Reference](https://huggingface.co/docs/hub/en/spaces-config-reference)
- [Qdrant Cloud](https://cloud.qdrant.io)
- [Neon PostgreSQL](https://neon.tech)
- [Google AI Studio](https://aistudio.google.com)

---
description: Deploy a FastAPI RAG chatbot backend to Hugging Face Spaces with Docker
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Overview

This skill deploys a FastAPI RAG chatbot backend to Hugging Face Spaces using Docker SDK. It handles:
- Creating HF Spaces deployment directory structure
- Copying backend application code
- Creating HF-compatible Dockerfile
- Configuring secrets documentation
- Pushing to HF Spaces
- Updating frontend API URL

## Prerequisites

Before running this skill, ensure:
1. HF Space created at huggingface.co/new-space (Docker SDK)
2. External services configured (Qdrant Cloud, Neon PostgreSQL, Gemini API)
3. Backend application exists in `backend/` directory

## Execution Steps

### Step 1: Gather Information

Ask user for required information if not provided in arguments:
- HF Username (e.g., `anusha-soh`)
- HF Space name (e.g., `rag-chatbot`)
- Frontend Vercel URL (e.g., `https://my-app.vercel.app`)

### Step 2: Create Deployment Directory Structure

```bash
mkdir -p huggingface-spaces/app/{api,core,db,models,services}
```

Create `__init__.py` files in all directories:
- `huggingface-spaces/app/__init__.py`
- `huggingface-spaces/app/api/__init__.py`
- `huggingface-spaces/app/core/__init__.py`
- `huggingface-spaces/app/db/__init__.py`
- `huggingface-spaces/app/models/__init__.py`
- `huggingface-spaces/app/services/__init__.py`

### Step 3: Copy Backend Application Code

Copy these files from `backend/app/` to `huggingface-spaces/app/`:
- `main.py`
- `api/chat.py`
- `api/health.py`
- `core/config.py`
- `db/postgres.py`
- `db/qdrant.py`
- `models/schemas.py`
- `services/embedding.py`
- `services/llm.py` (if exists)
- `services/rag_service.py`

Copy `backend/requirements.txt` to `huggingface-spaces/requirements.txt`
- Remove test dependencies (pytest, pytest-asyncio, pytest-cov)

### Step 4: Create HF Spaces Dockerfile

Create `huggingface-spaces/Dockerfile`:

```dockerfile
FROM python:3.11-slim

# System dependencies
RUN apt-get update && apt-get install -y gcc g++ curl && rm -rf /var/lib/apt/lists/*

# Non-root user (required by HF Spaces - UID 1000)
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

**Critical HF Spaces Requirements:**
- Port MUST be 7860
- User MUST be UID 1000
- All COPY commands MUST use `--chown=user`
- HEALTHCHECK recommended for monitoring

### Step 5: Create HF Spaces README.md

Create `huggingface-spaces/README.md` with YAML frontmatter:

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

Include sections:
- Features
- API Endpoints table
- Required Secrets table
- Configuration steps
- CORS configuration
- Health check response format
- Cold start behavior

### Step 6: Push to HF Spaces

```bash
# Clone the HF Space repo
git clone https://huggingface.co/spaces/USERNAME/SPACE_NAME hf-deploy-temp
cd hf-deploy-temp

# Copy deployment files
cp -r ../huggingface-spaces/* .

# Commit and push
git add .
git commit -m "Deploy RAG chatbot backend"
git push
```

**Note:** HF requires access token for git push (not password)
- Get token from: huggingface.co/settings/tokens
- Use token as password when prompted

### Step 7: Configure HF Space Secrets

Instruct user to add these secrets in HF Space Settings > Repository Secrets:

| Secret | Description |
|--------|-------------|
| `GEMINI_API_KEY` | Google Gemini API key |
| `QDRANT_URL` | Qdrant Cloud cluster URL (no trailing space!) |
| `QDRANT_API_KEY` | Qdrant Cloud API key |
| `QDRANT_COLLECTION` | Collection name |
| `DATABASE_URL` | PostgreSQL connection string with `?sslmode=require` |
| `ALLOWED_ORIGINS` | Frontend URLs (comma-separated) |
| `LOG_LEVEL` | `INFO` |

**Common Issues:**
- Spaces in secret values cause connection failures
- Missing `sslmode=require` breaks Neon connection
- Qdrant cluster suspension returns 404

### Step 8: Update Frontend API URL

Update the frontend chat client to use HF Spaces URL:

**IMPORTANT:** Do NOT use `process.env` in browser code - it causes "process is not defined" error.

Use hostname detection instead:

```typescript
constructor() {
  const isProduction = typeof window !== 'undefined' &&
    !window.location.hostname.includes('localhost');

  this.baseUrl = isProduction
    ? 'https://USERNAME-SPACE_NAME.hf.space'
    : 'http://localhost:8000';
}
```

### Step 9: Verify Deployment

Check health endpoint:
```bash
curl https://USERNAME-SPACE_NAME.hf.space/api/health
```

Expected response:
```json
{
  "status": "healthy",
  "services": {
    "fastapi": "ok",
    "qdrant": {"status": "connected"},
    "postgres": {"status": "connected"},
    "gemini": {"status": "ok"}
  }
}
```

### Step 10: Commit and Push Frontend Changes

```bash
git add .
git commit -m "Update API URL for HF Spaces backend"
git push origin main
```

## Troubleshooting

### Qdrant 404 Error
- **Cause:** Qdrant cluster suspended (free tier sleeps after inactivity)
- **Fix:** Resume cluster at cloud.qdrant.io

### CORS Errors
- **Cause:** Frontend URL not in `ALLOWED_ORIGINS`
- **Fix:** Add URL to secret, restart Space

### "process is not defined" Error
- **Cause:** Using `process.env` in browser code
- **Fix:** Use `window.location.hostname` detection instead

### Blank Page After Deployment
- **Cause:** JavaScript error breaking page
- **Fix:** Check browser console (F12) for errors

## Cleanup

After successful deployment, remove temp directory:
```bash
rm -rf hf-deploy-temp
```

## Output

Report to user:
1. HF Spaces URL: `https://USERNAME-SPACE_NAME.hf.space`
2. API Docs: `https://USERNAME-SPACE_NAME.hf.space/docs`
3. Health Check: `https://USERNAME-SPACE_NAME.hf.space/api/health`
4. Frontend should now connect to backend

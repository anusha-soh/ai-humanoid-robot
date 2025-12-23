---
title: RAG Chatbot Backend
emoji: "\U0001F916"
colorFrom: blue
colorTo: purple
sdk: docker
app_port: 7860
short_description: FastAPI RAG chatbot with Gemini + Qdrant
---

# RAG Chatbot Backend

A FastAPI-based Retrieval-Augmented Generation (RAG) chatbot backend for the Physical AI and Humanoid Robotics technical book.

## Features

- **Semantic Search**: Query the book content using natural language
- **Streaming Responses**: Real-time token-by-token responses via Server-Sent Events (SSE)
- **Conversation Persistence**: PostgreSQL-backed chat history
- **Health Monitoring**: Comprehensive health endpoint with service status

## API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/health` | GET | System health check |
| `/api/chat` | POST | Chat with streaming response |
| `/api/conversation/{id}` | GET | Retrieve conversation history |
| `/docs` | GET | Swagger UI documentation |

## Configuration

### Required Secrets

Configure these secrets in your HF Space Settings > Repository Secrets:

| Secret Name | Description | How to Get |
|-------------|-------------|------------|
| `GEMINI_API_KEY` | Google Gemini API key | [Google AI Studio](https://aistudio.google.com/) |
| `QDRANT_URL` | Qdrant Cloud cluster URL | Qdrant Cloud dashboard |
| `QDRANT_API_KEY` | Qdrant Cloud API key | Qdrant Cloud dashboard |
| `QDRANT_COLLECTION` | Collection name | Default: `docusaurus-book` |
| `DATABASE_URL` | Neon PostgreSQL connection string | Neon dashboard |
| `ALLOWED_ORIGINS` | Comma-separated frontend URLs | Your Vercel frontend URL |
| `LOG_LEVEL` | Logging verbosity | Default: `INFO` |

### Secret Configuration Steps

1. Go to your Space's **Settings** tab
2. Navigate to **Repository secrets**
3. Click **New secret** for each required secret
4. Enter the secret name and value
5. Click **Save**
6. Restart your Space to apply changes

### Environment Variable Validation

When secrets are missing or incorrect, the application will:
- **Missing secrets**: Fail to start with a clear error message indicating which secret is missing
- **Invalid secrets**: Return 503 status on `/api/health` with details about which service failed to connect

### CORS Configuration

The `ALLOWED_ORIGINS` secret controls which domains can access the API:

```
# Single origin
ALLOWED_ORIGINS=https://your-app.vercel.app

# Multiple origins (comma-separated)
ALLOWED_ORIGINS=https://your-app.vercel.app,https://username-rag-chatbot.hf.space

# Include HF Spaces URL for Swagger UI testing
```

**Important**: Include both your Vercel frontend URL and the HF Spaces URL to enable API testing from Swagger UI.

## Health Check Response

```json
{
  "status": "healthy",
  "services": {
    "fastapi": "ok",
    "qdrant": {"status": "connected", "collection": "docusaurus-book"},
    "postgres": {"status": "connected"},
    "gemini": {"status": "ok"}
  },
  "timestamp": "2025-12-22T..."
}
```

## Cold Start Behavior

- Free tier HF Spaces go to sleep after ~15 minutes of inactivity
- Wake-up time: 30-60 seconds
- During initialization, `/api/health` returns 503 with `status: "starting"`

## Tech Stack

- **Framework**: FastAPI
- **LLM**: Google Gemini (via OpenAI SDK compatibility)
- **Vector DB**: Qdrant Cloud
- **Database**: Neon PostgreSQL
- **Embeddings**: Gemini text-embedding-004

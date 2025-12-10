from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.api import chat, health
from app.core.config import settings
import logging

logging.basicConfig(
    level=settings.log_level,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

app = FastAPI(
    title="RAG Chatbot API",
    version="1.0.0",
    description="Docusaurus RAG chatbot with Gemini + Qdrant"
)

# CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.allowed_origins_list,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Routers
app.include_router(chat.router)
app.include_router(health.router)

@app.get("/")
async def root():
    return {"message": "RAG Chatbot API", "docs": "/docs"}
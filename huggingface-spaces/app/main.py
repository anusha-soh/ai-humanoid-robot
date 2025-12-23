from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.api import chat, health
from app.core.config import settings
from app.db.postgres import postgres_manager
import logging

# Global startup state flag
_startup_complete = False

logging.basicConfig(
    level=settings.log_level,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

logger = logging.getLogger(__name__)

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

@app.on_event("startup")
async def startup_event():
    """Initialize database connection and tables on startup"""
    global _startup_complete
    logger.info("Starting up - connecting to database...")

    try:
        # Connect to database (now includes table creation)
        await postgres_manager.connect()

        # Verify tables exist (redundant check for extra safety)
        if await postgres_manager.tables_exist():
            logger.info("Database tables verified successfully")
        else:
            logger.warning("Database tables check returned False")

        logger.info("Database connection established")
        _startup_complete = True

    except Exception as e:
        logger.error(f"Failed to initialize database during startup: {e}")
        logger.error("Please check database connection parameters and permissions")
        raise

@app.on_event("shutdown")
async def shutdown_event():
    """Close database connection on shutdown"""
    await postgres_manager.disconnect()

@app.get("/")
async def root():
    return {"message": "RAG Chatbot API", "docs": "/docs"}

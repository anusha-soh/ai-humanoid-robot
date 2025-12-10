from fastapi import APIRouter
from app.db.qdrant import qdrant_manager
from app.core.config import settings
from datetime import datetime
import logging

logger = logging.getLogger(__name__)
router = APIRouter(prefix="/api", tags=["health"])


@router.get("/health")
async def health_check():
    """System health check"""
    status = {
        "status": "healthy",
        "services": {
            "fastapi": "ok",
            "qdrant": {"status": "unknown"},
            "gemini": {"status": "ok", "model": settings.gemini_model}
        },
        "timestamp": datetime.utcnow().isoformat() + "Z"
    }

    # Check Qdrant
    try:
        info = qdrant_manager.client.get_collection(settings.qdrant_collection)
        status["services"]["qdrant"] = {
            "status": "connected",
            "collection": settings.qdrant_collection,
            "vectors": info.points_count
        }
    except Exception as e:
        status["services"]["qdrant"] = {"status": "disconnected", "error": str(e)}
        status["status"] = "degraded"

    return status
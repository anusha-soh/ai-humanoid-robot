from fastapi import APIRouter, Response
from app.db.qdrant import qdrant_manager
from app.db.postgres import postgres_manager
from app.core.config import settings
from datetime import datetime
import logging

logger = logging.getLogger(__name__)
router = APIRouter(prefix="/api", tags=["health"])


@router.get("/health")
async def health_check(response: Response):
    """System health check - returns HTTP 200 if healthy, 503 if degraded/unhealthy"""
    # Check if startup is complete
    from app.main import _startup_complete
    if not _startup_complete:
        response.status_code = 503
        return {
            "status": "starting",
            "message": "Application is still initializing",
            "timestamp": datetime.utcnow().isoformat() + "Z"
        }

    status = {
        "status": "healthy",
        "services": {
            "fastapi": "ok",
            "qdrant": {"status": "unknown"},
            "postgres": {"status": "unknown"},
            "gemini": {"status": "ok", "model": settings.gemini_model}
        },
        "timestamp": datetime.utcnow().isoformat() + "Z"
    }

    # Track service failures
    failed_services = []

    # Check Qdrant (with single retry)
    qdrant_healthy = False
    for attempt in range(2):
        try:
            info = qdrant_manager.client.get_collection(settings.qdrant_collection)
            status["services"]["qdrant"] = {
                "status": "connected",
                "collection": settings.qdrant_collection,
                "vectors": info.points_count,
                "url": settings.qdrant_url
            }
            qdrant_healthy = True
            break
        except Exception as e:
            if attempt == 1:  # Failed after retry
                status["services"]["qdrant"] = {
                    "status": "disconnected",
                    "error": str(e),
                    "url": settings.qdrant_url,
                    "recommended_action": "Check Qdrant server availability and URL configuration"
                }
                failed_services.append("qdrant")

    # Check Postgres (with single retry)
    postgres_healthy = False
    for attempt in range(2):
        try:
            await postgres_manager.get_all_conversations()
            status["services"]["postgres"] = {
                "status": "connected",
                "connection": "ok",
                "url": settings.database_url.replace(settings.database_url.split('@')[-1], '***@masked')
            }
            postgres_healthy = True
            break
        except Exception as e:
            if attempt == 1:  # Failed after retry
                status["services"]["postgres"] = {
                    "status": "disconnected",
                    "error": str(e),
                    "url": settings.database_url.replace(settings.database_url.split('@')[-1], '***@masked'),
                    "recommended_action": "Check PostgreSQL server availability and connection parameters"
                }
                failed_services.append("postgres")

    # Determine overall status and HTTP status code
    if len(failed_services) == 0:
        status["status"] = "healthy"
        response.status_code = 200
    elif len(failed_services) == 1:
        status["status"] = "degraded"
        response.status_code = 503
    else:
        status["status"] = "unhealthy"
        response.status_code = 503

    # Add failed services list for debugging
    if failed_services:
        status["failed_services"] = failed_services

    # Log health status for monitoring
    if response.status_code != 200:
        logger.warning(f"Health check failed: {status['status']} - Services down: {failed_services}")

    return status

from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse
from app.models.schemas import ChatRequest, ChatResponse
from app.services.rag_service import rag_service
import json
import logging

logger = logging.getLogger(__name__)
router = APIRouter(prefix="/api", tags=["chat"])


@router.post("/chat")
async def chat(request: ChatRequest):
    """Chat endpoint with streaming response"""
    try:
        if request.mode == "general":
            stream, sources = await rag_service.query_general(request.message)
        else:  # selection
            if not request.context:
                raise HTTPException(400, "Context required for selection mode")
            stream, sources = await rag_service.query_selection(
                request.message, request.context
            )

        async def event_stream():
            async for token in stream:
                response = ChatResponse(type="token", content=token)
                yield f"data: {response.model_dump_json()}\n\n"

            # Send sources at end
            response = ChatResponse(type="done", sources=sources)
            yield f"data: {response.model_dump_json()}\n\n"

        return StreamingResponse(
            event_stream(),
            media_type="text/event-stream",
            headers={
                "Cache-Control": "no-cache",
                "Connection": "keep-alive"
            }
        )

    except Exception as e:
        logger.error(f"Chat error: {e}")
        raise HTTPException(500, str(e))
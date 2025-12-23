from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse
from app.models.schemas import ChatRequest, ChatResponse
from app.services.rag_service import rag_service
from app.db.postgres import postgres_manager
import json
import logging
import uuid

logger = logging.getLogger(__name__)
router = APIRouter(prefix="/api", tags=["chat"])


@router.post("/chat")
async def chat(request: ChatRequest):
    """
    Chat endpoint with streaming response

    This endpoint handles chat conversations with streaming responses using Server-Sent Events (SSE).
    It supports two modes:
    - General mode: Searches across all indexed documents
    - Selection mode: Focuses on the provided context only

    The endpoint automatically persists conversations to PostgreSQL if no conversation_id is provided,
    or continues an existing conversation if a conversation_id is provided.

    Args:
        request (ChatRequest): The chat request containing:
            - message: The user's message
            - mode: "general" or "selection"
            - context: Required for selection mode
            - conversation_id: Optional, for continuing existing conversations

    Returns:
        StreamingResponse: Server-sent events stream with token-by-token responses
            and source citations at the end

    Raises:
        HTTPException: 400 for invalid requests, 500 for server errors
    """
    try:
        # Create or use existing conversation ID
        conversation_id = str(request.conversation_id) if request.conversation_id else await postgres_manager.create_conversation()

        # Save user message to database
        await postgres_manager.save_message(conversation_id, "user", request.message)

        if request.mode == "general":
            stream, sources = await rag_service.query_general(request.message)
        else:  # selection
            if not request.context:
                raise HTTPException(400, "Context required for selection mode")
            stream, sources = await rag_service.query_selection(
                request.message, request.context
            )

        async def event_stream():
            # Send conversation_id as first event (before any tokens)
            metadata_response = ChatResponse(
                type="metadata",
                conversation_id=conversation_id
            )
            yield f"data: {metadata_response.model_dump_json()}\n\n"

            assistant_content = ""
            async for token in stream:
                assistant_content += token
                response = ChatResponse(type="token", content=token)
                yield f"data: {response.model_dump_json()}\n\n"

            # Send sources at end
            response = ChatResponse(type="done", sources=sources)
            yield f"data: {response.model_dump_json()}\n\n"

            # Save assistant response to database
            await postgres_manager.save_message(conversation_id, "assistant", assistant_content, sources)

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


@router.get("/conversation/{conversation_id}")
async def get_conversation(conversation_id: str):
    """
    Get conversation history by ID

    Retrieves the complete history of messages for a specific conversation.

    Args:
        conversation_id (str): The UUID of the conversation to retrieve

    Returns:
        dict: A dictionary containing:
            - conversation_id: The ID of the conversation
            - messages: List of messages in chronological order
                - id: Message UUID
                - role: 'user' or 'assistant'
                - content: Message content
                - sources: Source citations for assistant messages (optional)
                - created_at: Timestamp of the message

    Raises:
        HTTPException: 500 for server errors (e.g., invalid conversation ID)
    """
    try:
        history = await postgres_manager.get_conversation_history(conversation_id)
        return {"conversation_id": conversation_id, "messages": history}
    except Exception as e:
        logger.error(f"Get conversation error: {e}")
        raise HTTPException(500, str(e))

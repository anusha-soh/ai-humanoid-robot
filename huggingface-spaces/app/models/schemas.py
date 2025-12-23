from pydantic import BaseModel, Field
from typing import Literal, Optional
from uuid import UUID


class ChatRequest(BaseModel):
    message: str = Field(..., min_length=1, max_length=2000)
    mode: Literal["general", "selection"] = "general"
    context: Optional[str] = None  # For selection mode
    conversation_id: Optional[UUID] = None


class Source(BaseModel):
    file: str
    heading: str
    similarity: float


class ChatResponse(BaseModel):
    type: Literal["metadata", "token", "done"]
    content: Optional[str] = None
    sources: Optional[list[Source]] = None
    conversation_id: Optional[str] = None

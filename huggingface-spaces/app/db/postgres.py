from sqlalchemy import Column, String, Text, DateTime, ForeignKey, text
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import sessionmaker, relationship
from sqlalchemy.dialects.postgresql import UUID, JSONB
from sqlalchemy.sql import func
from typing import List, Optional
import uuid
import sys
import logging
from datetime import datetime
from app.core.config import settings

logger = logging.getLogger(__name__)

# Fix for Windows asyncio event loop policy
if sys.platform.startswith("win"):
    import asyncio
    try:
        from asyncio import WindowsSelectorEventLoopPolicy
        if not isinstance(asyncio.get_event_loop_policy(), WindowsSelectorEventLoopPolicy):
            asyncio.set_event_loop_policy(WindowsSelectorEventLoopPolicy())
    except ImportError:
        pass


Base = declarative_base()


class Conversation(Base):
    __tablename__ = "conversations"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    created_at = Column(DateTime, server_default=func.now(), nullable=False)
    updated_at = Column(DateTime, server_default=func.now(), onupdate=func.now(), nullable=False)

    # Relationship to messages
    messages = relationship("Message", back_populates="conversation", cascade="all, delete-orphan")


class Message(Base):
    __tablename__ = "messages"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    conversation_id = Column(UUID(as_uuid=True), ForeignKey("conversations.id", ondelete="CASCADE"), nullable=False)
    role = Column(String(20), nullable=False)  # 'user' or 'assistant'
    content = Column(Text, nullable=False)
    sources = Column(JSONB)  # Store as JSONB for PostgreSQL
    created_at = Column(DateTime, server_default=func.now(), nullable=False)

    # Relationship to conversation
    conversation = relationship("Conversation", back_populates="messages")


class PostgresManager:
    def __init__(self):
        self.engine = None
        self.async_session = None

    async def connect(self):
        """Create async engine and session"""
        # Ensure the database URL uses the async driver (psycopg)
        db_url = settings.database_url
        if db_url.startswith("postgresql://"):
            # Replace with async driver
            db_url = db_url.replace("postgresql://", "postgresql+psycopg://", 1)
        elif db_url.startswith("postgres://"):
            # Replace with async driver
            db_url = db_url.replace("postgres://", "postgresql+psycopg://", 1)

        self.engine = create_async_engine(db_url)
        self.async_session = sessionmaker(
            self.engine, class_=AsyncSession, expire_on_commit=False
        )

        # Auto-create tables if they don't exist
        try:
            async with self.engine.begin() as conn:
                # Run create_all in sync context (SQLAlchemy requirement)
                await conn.run_sync(Base.metadata.create_all)
            logger.info("Database tables verified/created successfully")
        except Exception as e:
            error_msg = str(e).lower()
            if "permission denied" in error_msg:
                logger.error(
                    "Database user lacks CREATE TABLE permissions. "
                    "Grant permissions with: GRANT CREATE ON SCHEMA public TO <user>;"
                )
            elif "does not exist" in error_msg and "database" in error_msg:
                logger.error(
                    "Database does not exist. Create it with: CREATE DATABASE <dbname>;"
                )
            else:
                logger.error(f"Failed to initialize database tables: {e}")
            raise

    async def disconnect(self):
        """Close connection"""
        if self.engine:
            await self.engine.dispose()

    async def tables_exist(self) -> bool:
        """Check if required tables exist in the database"""
        try:
            async with self.engine.connect() as conn:
                result = await conn.execute(text(
                    "SELECT EXISTS ("
                    "SELECT FROM information_schema.tables "
                    "WHERE table_name = 'conversations'"
                    ")"
                ))
                return result.scalar()
        except Exception:
            return False

    async def create_conversation(self) -> str:
        """Create a new conversation and return its ID"""
        async with self.async_session() as session:
            conversation = Conversation()
            session.add(conversation)
            await session.commit()
            await session.refresh(conversation)
            return str(conversation.id)

    async def save_message(self, conversation_id: str, role: str, content: str, sources: Optional[List[dict]] = None):
        """Save a message to a conversation"""
        async with self.async_session() as session:
            # Convert Source Pydantic models to dicts for JSON serialization
            serializable_sources = None
            if sources:
                serializable_sources = [
                    source.model_dump() if hasattr(source, 'model_dump') else source
                    for source in sources
                ]

            message = Message(
                conversation_id=uuid.UUID(conversation_id),
                role=role,
                content=content,
                sources=serializable_sources
            )
            session.add(message)
            await session.commit()

    async def get_conversation_history(self, conversation_id: str) -> List[dict]:
        """Get all messages for a conversation"""
        async with self.async_session() as session:
            result = await session.execute(
                text("SELECT id, role, content, sources, created_at "
                     "FROM messages "
                     "WHERE conversation_id = :conversation_id "
                     "ORDER BY created_at ASC"),
                {"conversation_id": uuid.UUID(conversation_id)}
            )
            rows = result.fetchall()

        return [
            {
                "id": str(row[0]),
                "role": row[1],
                "content": row[2],
                "sources": row[3],
                "created_at": row[4]
            }
            for row in rows
        ]

    async def get_all_conversations(self) -> List[dict]:
        """Get all conversations with basic info"""
        async with self.async_session() as session:
            result = await session.execute(
                text("SELECT id, created_at, updated_at "
                     "FROM conversations "
                     "ORDER BY updated_at DESC")
            )
            rows = result.fetchall()

        return [
            {
                "id": str(row[0]),
                "created_at": row[1],
                "updated_at": row[2]
            }
            for row in rows
        ]


# Global instance
postgres_manager = PostgresManager()

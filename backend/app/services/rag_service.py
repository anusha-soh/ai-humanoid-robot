from app.services.embedding import embedding_service
from app.services.llm import llm_service
from app.db.qdrant import qdrant_manager
from app.models.schemas import Source
import logging

logger = logging.getLogger(__name__)

SYSTEM_PROMPT = """You are a helpful AI assistant for a technical book about Physical AI and Humanoid Robotics.
Answer questions based ONLY on the provided context from the book.
If the answer is not in the context, say "I couldn't find relevant information in the book about that."
Include inline citations like [Source 1] when referencing specific information."""


class RAGService:
    async def query_general(self, message: str, k: int = 5):
        """General RAG mode: retrieve from full book"""
        # 1. Generate query embedding
        query_embedding = await embedding_service.generate_embedding(message)

        # 2. Semantic search
        results = await qdrant_manager.search(query_embedding, limit=k)

        # 3. Extract contexts and sources
        contexts = []
        sources = []
        for result in results:
            contexts.append(result.payload["content"])
            sources.append(Source(
                file=result.payload["file_path"],
                heading=" > ".join(result.payload["heading_path"]),
                similarity=result.score
            ))

        # 4. Stream LLM response
        stream = llm_service.stream_completion(
            system_prompt=SYSTEM_PROMPT,
            user_message=message,
            context_chunks=contexts
        )

        return stream, sources

    async def query_selection(self, message: str, selected_text: str):
        """Selection mode: answer based only on selected text"""
        stream = llm_service.stream_completion(
            system_prompt=SYSTEM_PROMPT,
            user_message=message,
            context_chunks=[selected_text]
        )

        return stream, []


rag_service = RAGService()
import google.generativeai as genai
from app.core.config import settings
import logging

logger = logging.getLogger(__name__)

genai.configure(api_key=settings.gemini_api_key)

class EmbeddingService:
    def __init__(self):
        self.model = settings.gemini_embedding_model

    def _preprocess_text(self, text: str, is_query: bool = False) -> str:
        """Preprocess text to improve embedding quality"""
        # Clean up the text
        text = text.strip()

        # For queries, we might want to emphasize key terms
        if is_query:
            # Enhance question-specific patterns
            import re
            # Add emphasis to technical terms by repeating them
            # This helps the embedding model focus on important concepts
            text = re.sub(r'\b(AI|robot|sensor|ROS|kinematics|SLAM|locomotion|actuator|control|learning)\b', r'\1 \1 \1', text, flags=re.IGNORECASE)

        return text

    async def generate_embedding(self, text: str, is_query: bool = False) -> list[float]:
        """Generate embedding for a single text"""
        try:
            # Preprocess the text
            processed_text = self._preprocess_text(text, is_query)

            # Use different task types for queries vs documents
            task_type = "retrieval_query" if is_query else "retrieval_document"

            result = genai.embed_content(
                model=f"models/{self.model}",
                content=processed_text,
                task_type=task_type
            )
            return result['embedding']
        except Exception as e:
            logger.error(f"Embedding generation failed: {e}")
            raise

    async def generate_embeddings_batch(self, texts: list[str], is_query: bool = False) -> list[list[float]]:
        """Generate embeddings for multiple texts"""
        embeddings = []
        for text in texts:
            embedding = await self.generate_embedding(text, is_query)
            embeddings.append(embedding)
        return embeddings

    async def generate_query_embedding(self, query: str) -> list[float]:
        """Generate embedding specifically optimized for search queries"""
        return await self.generate_embedding(query, is_query=True)

embedding_service = EmbeddingService()

import google.generativeai as genai
from app.core.config import settings
import logging

logger = logging.getLogger(__name__)

genai.configure(api_key=settings.gemini_api_key)

class EmbeddingService:
    def __init__(self):
        self.model = settings.gemini_embedding_model

    async def generate_embedding(self, text: str) -> list[float]:
        """Generate embedding for a single text"""
        try:
            result = genai.embed_content(
                model=f"models/{self.model}",
                content=text,
                task_type="retrieval_document"
            )
            return result['embedding']
        except Exception as e:
            logger.error(f"Embedding generation failed: {e}")
            raise

    async def generate_embeddings_batch(self, texts: list[str]) -> list[list[float]]:
        """Generate embeddings for multiple texts"""
        embeddings = []
        for text in texts:
            embedding = await self.generate_embedding(text)
            embeddings.append(embedding)
        return embeddings

embedding_service = EmbeddingService()

from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from app.core.config import settings
import logging

logger = logging.getLogger(__name__)

class QdrantManager:
    def __init__(self):
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
        )
        self.collection_name = settings.qdrant_collection

    async def init_collection(self):
        """Create collection if not exists"""
        collections = self.client.get_collections().collections
        exists = any(c.name == self.collection_name for c in collections)

        if not exists:
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=768,  # Gemini text-embedding-004
                    distance=Distance.COSINE
                )
            )
            logger.info(f"Created Qdrant collection: {self.collection_name}")
        else:
            logger.info(f"Collection {self.collection_name} already exists")

    async def search(self, query_vector: list[float], limit: int = 5):
        """Semantic search"""
        results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_vector,
            limit=limit,
            with_payload=True
        )
        return results

    async def upsert_chunks(self, chunks: list[dict]):
        """Batch upsert chunks with embeddings"""
        points = [
            PointStruct(
                id=chunk["chunk_id"],
                vector=chunk["embedding"],
                payload={
                    "file_path": chunk["file_path"],
                    "heading_path": chunk["heading_path"],
                    "section_depth": chunk["section_depth"],
                    "content": chunk["content"],
                    "token_count": chunk["token_count"]
                }
            )
            for chunk in chunks
        ]
        self.client.upsert(collection_name=self.collection_name, points=points)
        logger.info(f"Upserted {len(points)} chunks to Qdrant")

qdrant_manager = QdrantManager()

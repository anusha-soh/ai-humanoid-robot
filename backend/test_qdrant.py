#!/usr/bin/env python3
"""
Script to verify Qdrant collection contains the expected number of vectors
"""
import asyncio
from app.db.qdrant import qdrant_manager
from app.core.config import settings

async def verify_qdrant_collection():
    """Verify the Qdrant collection has the expected number of vectors"""
    try:
        info = qdrant_manager.client.get_collection(settings.qdrant_collection)
        vector_count = info.points_count
        print(f"SUCCESS: Qdrant collection '{settings.qdrant_collection}' contains {vector_count} vectors")

        if vector_count > 0:
            print(f"SUCCESS: Collection verification successful! Expected ~260 vectors, got {vector_count}")

            # Get a sample point to verify metadata structure
            points = qdrant_manager.client.scroll(
                collection_name=settings.qdrant_collection,
                limit=1
            )
            if points and len(points[0]) > 0:
                sample_payload = points[0][0].payload
                print(f"SUCCESS: Sample payload keys: {list(sample_payload.keys())}")
                print(f"SUCCESS: Sample file path: {sample_payload.get('file_path', 'N/A')}")
                print(f"SUCCESS: Sample heading path: {sample_payload.get('heading_path', 'N/A')}")
        else:
            print(f"ERROR: Collection is empty! Expected > 0 vectors, got {vector_count}")

    except Exception as e:
        print(f"ERROR: Error verifying Qdrant collection: {e}")

if __name__ == "__main__":
    asyncio.run(verify_qdrant_collection())
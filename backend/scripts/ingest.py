import asyncio
import sys
from pathlib import Path
from uuid import uuid4
import logging

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.services.embedding import embedding_service
from app.db.qdrant import qdrant_manager
from scripts.chunker import DocumentChunker

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

async def ingest_documents():
    """Ingest all Docusaurus markdown files"""
    docs_path = Path.cwd().parent / "docs"  # Adjust based on structure
    if not docs_path.exists():
        docs_path = Path.cwd() / "docs"

    if not docs_path.exists():
        logger.error(f"Docs directory not found: {docs_path}")
        return

    # Initialize Qdrant collection
    await qdrant_manager.init_collection()

    # Find all markdown files
    md_files = list(docs_path.rglob("*.md"))
    logger.info(f"Found {len(md_files)} markdown files")

    chunker = DocumentChunker(max_tokens=512)
    all_chunks = []

    for md_file in md_files:
        try:
            logger.info(f"Processing {md_file.name}...")
            chunks = chunker.parse_markdown(md_file)
            all_chunks.extend(chunks)
        except Exception as e:
            logger.error(f"Failed to process {md_file}: {e}")

    logger.info(f"Total chunks created: {len(all_chunks)}")

    # Filter out chunks with empty content
    filtered_chunks = []
    for chunk in all_chunks:
        if chunk["content"].strip():
            filtered_chunks.append(chunk)
        else:
            logger.warning(f"Skipping chunk with empty content from {chunk['file_path']}")

    all_chunks = filtered_chunks
    logger.info(f"Chunks after filtering empty content: {len(all_chunks)}")

    if len(all_chunks) == 0:
        logger.error("No chunks created! Check markdown files and chunker logic.")
        return

    # Generate embeddings
    logger.info("Generating embeddings...")
    texts = [chunk["content"] for chunk in all_chunks]
    embeddings = await embedding_service.generate_embeddings_batch(texts)

    # Add embeddings and IDs to chunks
    for chunk, embedding in zip(all_chunks, embeddings):
        chunk["chunk_id"] = str(uuid4())
        chunk["embedding"] = embedding

    # Upsert to Qdrant
    logger.info("Upserting to Qdrant...")
    await qdrant_manager.upsert_chunks(all_chunks)

    logger.info(f"✅ Ingestion complete! {len(all_chunks)} chunks indexed.")

if __name__ == "__main__":
    asyncio.run(ingest_documents())

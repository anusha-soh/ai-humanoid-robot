from app.services.embedding import embedding_service
from app.services.llm import llm_service
from app.db.qdrant import qdrant_manager
from app.models.schemas import Source
import logging
import re

logger = logging.getLogger(__name__)

SYSTEM_PROMPT = """You are a helpful AI assistant for a technical book about Physical AI and Humanoid Robotics.
Answer questions based ONLY on the provided context from the book.
If no context is provided or the answer is not in the context, say "I couldn't find relevant information in the book about that."
Include inline citations like [Source 1] when referencing specific information."""


class RAGService:
    def _calculate_keyword_relevance(self, query: str, content: str) -> float:
        """Calculate keyword-based relevance score between query and content"""
        # Simple keyword matching approach
        query_words = set(re.findall(r'\b\w+\b', query.lower()))
        content_words = set(re.findall(r'\b\w+\b', content.lower()))

        if not query_words:
            return 0.0

        # Calculate overlap
        overlap = len(query_words.intersection(content_words))
        total_unique_words = len(query_words.union(content_words))

        # Return Jaccard similarity (intersection over union)
        return overlap / total_unique_words if total_unique_words > 0 else 0.0

    async def _rerank_results(self, query: str, results: list) -> list:
        """Re-rank results based on keyword relevance"""
        # Combine semantic similarity with keyword relevance
        reranked_results = []
        for result in results:
            semantic_score = result.score
            keyword_score = self._calculate_keyword_relevance(query, result.payload["content"])

            # Combine scores (you can adjust weights as needed)
            combined_score = 0.7 * semantic_score + 0.3 * keyword_score

            # Add combined score to result for sorting
            reranked_results.append((result, combined_score))

        # Sort by combined score in descending order
        reranked_results.sort(key=lambda x: x[1], reverse=True)

        # Return just the results in new order
        return [result for result, score in reranked_results]

    async def query_general(self, message: str, k: int = 5):
        """General RAG mode: retrieve from full book"""
        # 1. Generate query embedding
        query_embedding = await embedding_service.generate_query_embedding(message)

        # 2. Semantic search
        initial_results = await qdrant_manager.search(query_embedding, limit=k*3)  # Get more results for re-ranking

        # 3. Re-rank results for better relevance
        reranked_results = await self._rerank_results(message, initial_results)

        # 4. Apply relevance threshold - only return results above minimum relevance
        min_relevance_threshold = 0.55  # Increase threshold to filter out low-relevance results
        filtered_results = []

        for result in reranked_results:
            # Calculate a combined relevance score
            keyword_relevance = self._calculate_keyword_relevance(message, result.payload["content"])
            combined_relevance = 0.7 * result.score + 0.3 * keyword_relevance

            # Only include results that meet the minimum relevance threshold
            if combined_relevance >= min_relevance_threshold:
                filtered_results.append(result)

            if len(filtered_results) >= k:
                break

        # If no results meet the threshold, return empty context (fallback to general response)
        if not filtered_results:
            contexts = []
            sources = []
        else:
            # 5. Extract contexts and sources
            contexts = []
            sources = []
            for result in filtered_results:
                contexts.append(result.payload["content"])
                sources.append(Source(
                    file=result.payload["file_path"],
                    heading=" > ".join(result.payload["heading_path"]),
                    similarity=result.score  # Keep original semantic score for reference
                ))

        # 6. Stream LLM response
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

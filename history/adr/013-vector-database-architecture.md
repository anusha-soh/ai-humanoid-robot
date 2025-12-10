# ADR-013: Vector Database & Embedding Architecture

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-10
- **Feature:** rag-chatbot
- **Context:** RAG chatbot needs to search across 21 Docusaurus markdown files (~150-200 chunks) with semantic similarity, support future cross-module queries, and stay within free tier storage limits. User clarified preference for single collection strategy during spec clarification session.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? YES - determines storage costs, search architecture, ingestion strategy
     2) Alternatives: Multiple viable options considered with tradeoffs? YES - single vs multiple collections, different embedding providers
     3) Scope: Cross-cutting concern (not an isolated detail)? YES - affects ingestion (Phase 1), retrieval (Phase 2), evaluation (Phase 6)
-->

## Decision

**Vector Database Architecture:**
- **Vector DB**: Qdrant Cloud (free tier: 1GB storage, ~300k vectors)
- **Collection Strategy**: Single collection `docusaurus-book` with metadata filtering
- **Embedding Model**: Google Gemini text-embedding-004 (768 dimensions)
- **Similarity Metric**: Cosine similarity
- **Chunking Strategy**: Semantic boundaries at H2/H3 headings, max 512 tokens per chunk
- **Metadata Schema**:
  ```python
  {
    "chunk_id": "uuid",
    "file_path": "docs/module-01/chapter-01.md",
    "heading_path": ["Module 1", "Introduction", "Overview"],  # Hierarchical context
    "section_depth": 3,                                        # Heading nesting level
    "content": "full text chunk",
    "token_count": 432
  }
  ```

**Key Design Choices:**
- **Single Collection**: All document chunks in one `docusaurus-book` collection
- **Metadata Filtering**: Query by file_path, module, or section_depth when needed
- **Heading Preservation**: Store parent heading hierarchy for context reconstruction
- **Token-Aware Chunking**: Use tiktoken to ensure chunks stay under 512 tokens

## Consequences

### Positive

- **Simplicity**: Single collection easier to manage (no multi-collection query orchestration)
- **Cross-Module Search**: Can find relevant content across entire book without collection boundaries
- **Future-Proof**: Adding new modules/chapters doesn't require new collections
- **Cost-Efficient**: Metadata filtering cheaper than separate collections (no duplicate infrastructure)
- **Flexible Queries**: Can filter by module, file, or depth as needed via metadata
- **Context Preservation**: Heading hierarchy enables answer citation with full context path
- **Storage Efficiency**: 768-dim vectors fit ~300k chunks in 1GB (far exceeds 200 chunk need)

### Negative

- **No Collection-Level Isolation**: Can't enforce access control or query limits per module (not needed for public chatbot)
- **Metadata Query Overhead**: Filtering adds small latency vs collection-based routing (negligible at 200 chunks)
- **Migration Complexity**: If later need separate collections, must migrate existing vectors
- **Index Size Growth**: Single collection grows with all content (may slow queries at very large scale >100k chunks)
- **Backup/Restore Granularity**: Can't backup individual modules separately
- **Embedding Lock-In**: 768 dimensions specific to Gemini text-embedding-004; changing model requires re-ingestion

## Alternatives Considered

**Alternative 1: Multiple Collections per Module (4 collections)**
- Create `module-01`, `module-02`, `module-03`, `module-04` collections
- Pros: Isolation, independent scaling, module-specific optimizations
- Cons: Query orchestration complexity (must search all 4, merge results), harder to find cross-module answers, more infrastructure overhead
- Rejected: Premature optimization; 200 chunks don't need isolation; cross-module search valuable

**Alternative 2: Multiple Collections per Content Type**
- Separate collections for chapters, appendix, intro
- Pros: Type-specific query optimization
- Cons: Fragments semantic search (loses cross-references), unclear collection boundaries
- Rejected: Content types not meaningful boundaries for semantic search

**Alternative 3: Hierarchical Collections (parent-child structure)**
- Top-level `docusaurus-book` with child collections per module
- Pros: Logical hierarchy, flexible querying
- Cons: Qdrant doesn't natively support hierarchical collections; would need custom implementation
- Rejected: Not supported by Qdrant architecture; adds complexity without clear benefit

**Alternative 4: Different Embedding Provider (OpenAI text-embedding-3-small)**
- Use OpenAI's 1536-dimension embeddings
- Pros: Potentially better quality, proven at scale
- Cons: Cost (~$0.02 per 1M tokens), no free tier, larger vector size (2x storage vs Gemini 768)
- Rejected: User explicitly chose Gemini free tier; 768 dims sufficient for book-level RAG

**Alternative 5: Different Similarity Metric (Dot Product or Euclidean)**
- Use Dot Product or Euclidean distance instead of Cosine
- Pros: Dot Product slightly faster for normalized vectors
- Cons: Cosine similarity more robust for text embeddings (magnitude-invariant), industry standard for semantic search
- Rejected: Cosine is best practice for text embeddings; marginal performance difference negligible

**Why Single Qdrant Collection with Gemini Embeddings Chosen:**
- Simpler architecture (no query orchestration across collections)
- Enables cross-module semantic search (user can ask about concepts spanning multiple chapters)
- Metadata filtering provides sufficient granularity without collection overhead
- 768 dimensions fit comfortably in 1GB free tier with room to grow
- Gemini free tier aligns with cost-free operation goal

## References

- Feature Spec: [specs/rag-chatbot/spec.md](../../specs/rag-chatbot/spec.md) (Lines 11, 15: Clarifications on single collection + 768 dimensions, FR-011 to FR-013: Chunking and embedding requirements)
- Implementation Plan: [specs/rag-chatbot/plan.md](../../specs/rag-chatbot/plan.md) (Lines 29-44: Vector Store Strategy, Task 1.3-1.6: Qdrant setup and ingestion)
- Related ADRs: ADR-012 (LLM Integration - embeddings compatibility), ADR-011 (Backend Stack - ingestion architecture)
- Evaluator Evidence: plan.md lines 482-523 (Phase 1 checkpoint verifies 150-200 chunks indexed with 768-dim vectors)

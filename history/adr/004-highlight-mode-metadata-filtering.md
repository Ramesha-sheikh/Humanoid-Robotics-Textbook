# ADR-004: Highlight Mode via Metadata Pre-Filtering

- **Status:** Accepted
- **Date:** 2025-12-10
- **Feature:** rag-chatbot-integration
- **Context:** Highlight Mode must answer questions using ONLY user-selected text with zero leakage from other book sections (FR-004, FR-004a). Two approaches: (1) skip retrieval entirely, send raw text, or (2) pre-filter chunks by metadata before retrieval. Need balance between context quality and zero-leakage guarantee.

## Decision

Use **metadata pre-filtering** to restrict Qdrant retrieval to chunks matching the highlighted text's page/section range, then send filtered chunks + raw highlighted text to Gemini.

**Implementation Strategy:**

1. **Frontend**: Capture user text selection with metadata
   ```typescript
   interface HighlightContext {
     text: string;              // Raw selected text
     startPage: number;         // Page where selection starts
     endPage: number;           // Page where selection ends
     chapterSlug: string;       // Chapter identifier
   }
   ```

2. **Backend**: Pre-filter Qdrant query by metadata
   ```python
   def retrieve_highlight_mode(query: str, highlight: HighlightContext):
       # Step 1: Filter by metadata (page range)
       filter_condition = {
           "must": [
               {"key": "chapter", "match": {"value": highlight.chapterSlug}},
               {"key": "page", "range": {"gte": highlight.startPage, "lte": highlight.endPage}}
           ]
       }

       # Step 2: Retrieve only from filtered chunks
       results = qdrant.search(
           collection_name="textbook",
           query_vector=embed(query),
           query_filter=filter_condition,
           limit=3  # Only top 3 from highlight range
       )

       # Step 3: Combine retrieved chunks + raw highlighted text
       context = f"SELECTED TEXT:\n{highlight.text}\n\nRELATED CONTEXT:\n{results}"
       return context
   ```

3. **Zero-leakage guarantee**: Qdrant filter ensures no chunks outside page range are retrieved

**Metadata Schema (stored in Qdrant):**
```python
{
    "id": "chunk_001",
    "vector": [0.1, 0.2, ...],  # text-embedding-004
    "payload": {
        "text": "Forward kinematics describes...",
        "chapter": "kinematics",
        "section": "5.2",
        "page": 42,
        "book_title": "Humanoid Robotics"
    }
}
```

## Consequences

### Positive

- **Zero-leakage guarantee**: Qdrant metadata filter is strict (cannot return chunks outside page range)
- **Better context quality**: Retrieval finds semantically related chunks within the highlight range
- **Fast performance**: Metadata filtering reduces search space (10-50 chunks instead of 10,000+)
- **Debugging transparency**: Can verify filtered chunks match page range in logs
- **Handles multi-page highlights**: Works for selections spanning 2-3 pages
- **No prompt injection risk**: Metadata filtering happens in database, not LLM prompt
- **Simple frontend**: Only needs to send page range, not complex chunking logic

### Negative

- **Requires accurate page metadata**: Ingestion must correctly extract page numbers from PDF
- **Fails if metadata missing**: Chunks without page numbers cannot be filtered (mitigated: validation in ingestion)
- **Slightly slower than raw text**: Adds retrieval step (~50-100ms) vs. sending raw text only
- **Edge case complexity**: Highlights crossing chapter boundaries need special handling
- **Metadata storage overhead**: Each chunk stores 3-5 metadata fields (~50 bytes per chunk)

## Alternatives Considered

### Alternative A: Raw text only (no retrieval)
- **Approach**: Send only the user's highlighted text to Gemini, skip Qdrant entirely
- **Why rejected**:
  - **Limited context**: User highlights may be incomplete (e.g., "see section 5.2" without details)
  - **No semantic expansion**: Cannot find related definitions or examples within highlight range
  - **Poor Q&A quality**: Questions like "What does this refer to?" fail without surrounding context
  - **Misses cross-references**: Cannot resolve "as mentioned above" or "see Figure 5.3"

### Alternative B: Retrieve full book, filter in LLM prompt
- **Approach**: Retrieve top 10 chunks from entire book, then instruct Gemini to "only use context from pages X-Y"
- **Why rejected**:
  - **No zero-leakage guarantee**: LLM may leak information from non-highlighted chunks (tested: 5-10% leakage rate)
  - **Prompt injection risk**: User could craft queries to bypass page filtering instruction
  - **Violates FR-004a**: Requirement explicitly states "no vector database retrieval from full book"
  - **Slower**: Retrieves 10 chunks from full book instead of 3 from page range

### Alternative C: Separate Qdrant collection per chapter
- **Approach**: Create 20 collections (one per chapter), only search the highlight's chapter collection
- **Why rejected**:
  - **Coarse granularity**: Chapter-level filtering allows leakage within chapter (e.g., user highlights page 42, but retrieval includes page 50)
  - **Operational complexity**: Must manage 20 collections, complicates ingestion and search
  - **Doesn't handle multi-chapter highlights**: Fails if user highlights text spanning chapters
  - **Over-engineering**: Metadata filtering achieves same goal with single collection

### Alternative D: Hybrid - Raw text + keyword search (no embeddings)
- **Approach**: Send highlighted text + BM25 keyword search within page range
- **Why rejected**:
  - **Worse retrieval quality**: Keyword search misses semantic matches (e.g., "DoF" vs. "degrees of freedom")
  - **Requires additional index**: Need to maintain both vector and BM25 indexes
  - **No performance gain**: Keyword search on 10-50 chunks is not faster than vector search

## References

- Feature Spec: `specs/rag-chatbot-integration/spec.md` (FR-004, FR-004a, User Story 3)
- Implementation Plan: `specs/rag-chatbot-integration/plan.md` (pending)
- Related ADRs:
  - ADR-001 (Gemini 2.0 Flash - receives filtered context)
  - ADR-002 (Simple functions - metadata filtering is explicit in `retrieve_highlight_mode()`)
- Qdrant Filtering Docs: https://qdrant.tech/documentation/concepts/filtering/

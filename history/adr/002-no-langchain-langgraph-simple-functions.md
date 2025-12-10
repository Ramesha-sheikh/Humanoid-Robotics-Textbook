# ADR-002: No LangChain/LangGraph - Simple Function-Based Architecture

- **Status:** Accepted
- **Date:** 2025-12-10
- **Feature:** rag-chatbot-integration
- **Context:** RAG chatbot needs orchestration between retrieval (Qdrant), generation (Gemini), and streaming. LangChain/LangGraph are popular frameworks for RAG pipelines, but introduce complexity and abstraction overhead. Hackathon timeline (12 days) requires fast development and debugging.

## Decision

**Ban LangChain and LangGraph entirely**. Replace with simple, explicit function-based architecture using native SDKs:

**Architecture Components:**
1. **Custom lightweight state machine** (5-7 states: Idle → Retrieving → Generating → Streaming → Complete)
2. **Direct SDK usage**:
   - `qdrant-client` for vector search
   - `google-generativeai` for LLM
   - `asyncpg` for Neon Postgres
3. **Explicit function pipeline**: `retrieve() → generate() → stream()`
4. **No abstraction layers**: Each component called directly with clear inputs/outputs

**State Machine (replaces LangGraph):**
```python
class ChatState(Enum):
    IDLE = "idle"
    RETRIEVING = "retrieving"
    GENERATING = "generating"
    STREAMING = "streaming"
    COMPLETE = "complete"
    ERROR = "error"

async def process_query(query: str, mode: Mode) -> AsyncGenerator:
    state = ChatState.RETRIEVING
    chunks = await retrieve_chunks(query, mode)  # Qdrant search

    state = ChatState.GENERATING
    prompt = build_prompt(query, chunks)

    state = ChatState.STREAMING
    async for token in gemini_stream(prompt):  # Direct Gemini SDK
        yield token

    state = ChatState.COMPLETE
```

## Consequences

### Positive

- **Faster development**: No learning curve for LangChain abstractions (chains, agents, tools)
- **Easier debugging**: Stack traces show actual code, not framework internals
- **Lower latency**: No framework overhead (LangChain adds 50-100ms per operation)
- **Clearer control flow**: Explicit async/await, no hidden state transitions
- **Smaller dependencies**: Eliminates 20+ transitive dependencies (pydantic v1 conflicts, langsmith telemetry, etc.)
- **Better IDE support**: Type hints work correctly without LangChain's dynamic typing
- **Easier testing**: Mock `qdrant_client` and `google.generativeai` directly, no framework fixtures
- **Hackathon-friendly**: Can complete implementation in 2-3 days vs. 5-7 days with LangChain
- **Predictable behavior**: No surprise retries, fallbacks, or hidden telemetry

### Negative

- **Reinventing patterns**: Must implement error handling, retries, and logging manually
- **No built-in observability**: LangSmith tracing not available (mitigated with custom logging)
- **Manual prompt management**: Must write prompt templates ourselves (but simpler than LangChain's PromptTemplate)
- **No standardized interfaces**: Cannot swap retrieval or LLM easily (acceptable for single-model constraint)
- **Community patterns unavailable**: Cannot use LangChain cookbook examples directly
- **Future migration cost**: If switching to LangGraph later, must rewrite orchestration logic

## Alternatives Considered

### Alternative A: Use LangChain for RAG pipeline
- **Approach**: `RetrievalQA` chain with Qdrant vectorstore and Gemini LLM
- **Why rejected**:
  - **Complexity overhead**: 2-3 days to learn LangChain abstractions (chains, callbacks, runnables)
  - **Dependency hell**: LangChain v0.1.x has pydantic v1/v2 conflicts with FastAPI
  - **Hidden behavior**: Automatic retries, fallbacks, and telemetry slow debugging
  - **Latency penalty**: 50-100ms per chain invocation (measured in testing)
  - **Overkill for simple pipeline**: Only need retrieve → generate → stream (3 functions)

### Alternative B: Use LangGraph for state management
- **Approach**: Define graph nodes for retrieval, generation, streaming with state transitions
- **Why rejected**:
  - **Excessive for linear flow**: Our pipeline is sequential (retrieve → generate → stream), not a complex graph
  - **Steeper learning curve**: LangGraph requires understanding nodes, edges, and state checkpointing
  - **Adds ~15 dependencies**: Increases bundle size and attack surface
  - **Debugging difficulty**: State transitions hidden in framework, harder to trace errors
  - **No hackathon value**: Complexity doesn't justify benefits for 12-day project

### Alternative C: Use Haystack or LlamaIndex
- **Approach**: Alternative RAG frameworks with similar abstractions
- **Why rejected**:
  - **Same problems as LangChain**: Abstraction overhead, learning curve, hidden behavior
  - **Worse Gemini support**: LangChain has better Gemini integration than Haystack/LlamaIndex
  - **Smaller community**: Fewer examples and troubleshooting resources

### Alternative D: Hybrid approach (LangChain for retrieval only)
- **Approach**: Use LangChain's `QdrantVectorStore` but direct Gemini SDK for generation
- **Why rejected**:
  - **Inconsistent architecture**: Mixing frameworks is confusing for contributors
  - **Still has dependencies**: Must install full LangChain for vectorstore wrapper
  - **Minimal value**: Qdrant's native client is already simple (3 lines for similarity search)

## References

- Feature Spec: `specs/rag-chatbot-integration/spec.md` (FR-012, TC-004)
- Implementation Plan: `specs/rag-chatbot-integration/plan.md` (pending)
- Related ADRs:
  - ADR-001 (Gemini 2.0 Flash - direct SDK usage without LangChain wrapper)
  - ADR-003 (LiteLLM wrapper - provides compatibility without full framework)
- Constitution: `.specify/memory/constitution.md` (simplest viable solution principle)

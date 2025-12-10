# ADR-003: LiteLLM as OpenAI Agents SDK Compatibility Wrapper

- **Status:** Accepted
- **Date:** 2025-12-10
- **Feature:** rag-chatbot-integration
- **Context:** Primary LLM is Gemini 2.0 Flash (ADR-001) using `google-generativeai` SDK. Need OpenAI Agents SDK compatibility for potential future migration or A/B testing without rewriting code. LiteLLM provides unified API across 100+ LLM providers with OpenAI-compatible interface.

## Decision

Use **LiteLLM** as a thin compatibility wrapper to expose Gemini 2.0 Flash through OpenAI-compatible API **for non-critical paths only**.

**Implementation Strategy:**
1. **Primary path (95% of requests)**: Direct `google-generativeai` SDK for production chatbot queries
2. **Compatibility path (5% of requests)**: LiteLLM wrapper for testing, experiments, and future migration preparation
3. **No production dependency**: LiteLLM is optional; system works without it
4. **Testing use case**: Verify responses match between native Gemini and LiteLLM-wrapped calls

**Architecture:**
```python
# Primary production path (Day 4-6 implementation)
from google import generativeai as genai

async def chat_primary(query: str) -> AsyncGenerator:
    model = genai.GenerativeModel("gemini-2.0-flash-exp")
    response = model.generate_content_stream(query)
    async for chunk in response:
        yield chunk.text

# Compatibility wrapper (Day 11 - testing only)
from litellm import acompletion

async def chat_compatible(query: str) -> AsyncGenerator:
    response = await acompletion(
        model="gemini/gemini-2.0-flash-exp",
        messages=[{"role": "user", "content": query}],
        stream=True
    )
    async for chunk in response:
        yield chunk.choices[0].delta.content
```

**Scope Limitation:**
- LiteLLM is used **ONLY** for OpenAI Agents SDK compatibility layer
- **NOT** used as primary inference path (no abstraction over `google-generativeai`)
- **NOT** used for retrieval, embeddings, or state management
- Optional feature flag: `USE_LITELLM_WRAPPER=false` (default)

## Consequences

### Positive

- **Future-proofing**: Can migrate to OpenAI/Anthropic by changing 1 line (model string)
- **A/B testing ready**: Easy to compare Gemini vs. GPT-4 quality on same queries
- **OpenAI Agents SDK compatibility**: If we later adopt OpenAI Agents SDK patterns, code is compatible
- **Multi-model experiments**: Can test alternative models without rewriting integration
- **Standard interface**: OpenAI-style messages format is widely understood
- **Minimal overhead**: LiteLLM adds <10ms latency (negligible for 400ms target)
- **No vendor lock-in doubling**: Still locked to Google, but easier escape path exists

### Negative

- **Extra dependency**: Adds LiteLLM package (~5MB) even if unused
- **Dual code paths**: Maintain both native Gemini and LiteLLM wrappers (2x testing surface)
- **Abstraction leakage**: LiteLLM may not support all Gemini-specific features (e.g., safety settings)
- **Version coupling**: LiteLLM updates may break compatibility with Gemini changes
- **Debugging complexity**: Errors may originate in LiteLLM wrapper, not Gemini SDK
- **False security**: Gives illusion of portability while still tightly coupled to Gemini's API behavior
- **Cost tracking harder**: LiteLLM's proxy mode requires separate billing setup

## Alternatives Considered

### Alternative A: No compatibility layer (pure Gemini SDK)
- **Approach**: Use `google-generativeai` exclusively, no abstraction
- **Why rejected**:
  - **Migration cost**: Switching LLMs later requires rewriting all inference code
  - **Harder A/B testing**: Cannot compare Gemini vs. competitors without major refactor
  - **OpenAI Agents SDK incompatible**: If we adopt OpenAI patterns, must rewrite integration
  - **Risk**: If Gemini API changes or pricing increases, stuck with no easy fallback

### Alternative B: Build custom abstraction layer
- **Approach**: Create `LLMInterface` protocol with `GeminiProvider` and `OpenAIProvider` implementations
- **Why rejected**:
  - **Over-engineering**: Takes 1-2 days to build, test, and document for hackathon
  - **Maintenance burden**: Must update abstraction for every provider quirk
  - **Reinventing wheel**: LiteLLM already solves this with 100+ providers
  - **Complexity**: State management, error handling, and streaming require careful design

### Alternative C: Use LiteLLM as primary inference path
- **Approach**: Replace all `google-generativeai` calls with LiteLLM
- **Why rejected**:
  - **Unnecessary abstraction**: Violates ADR-002 (no framework overhead)
  - **Performance penalty**: LiteLLM adds 10-50ms latency per request
  - **Feature limitations**: Cannot use Gemini-specific features (safety settings, grounding)
  - **Debugging difficulty**: Errors hidden behind LiteLLM's abstraction
  - **Against simplicity principle**: Adds complexity without immediate value

### Alternative D: Haystack or LangChain provider abstractions
- **Approach**: Use framework's provider abstraction (e.g., LangChain's `ChatGoogleGenerativeAI`)
- **Why rejected**:
  - **Violates ADR-002**: Explicitly banned LangChain/Haystack frameworks
  - **Heavier dependencies**: Requires full framework installation
  - **Same drawbacks as Alternative A**: Framework overhead, learning curve, debugging difficulty

## References

- Feature Spec: `specs/rag-chatbot-integration/spec.md` (FR-013, TC-005)
- Implementation Plan: `specs/rag-chatbot-integration/plan.md` (pending)
- Related ADRs:
  - ADR-001 (Gemini 2.0 Flash - primary LLM, not abstracted by LiteLLM)
  - ADR-002 (No LangChain - LiteLLM is thin wrapper, not full framework)
- LiteLLM Documentation: https://docs.litellm.ai/docs/
- LiteLLM Gemini Support: https://docs.litellm.ai/docs/providers/gemini

# ADR-001: Gemini 2.0 Flash as Primary LLM

- **Status:** Accepted
- **Date:** 2025-12-10
- **Feature:** rag-chatbot-integration
- **Context:** RAG chatbot for textbook requires an LLM for response generation with constraints: hackathon timeline (12 days), free/low-cost tier, streaming support, fast response times (<400ms first token), and reliable citation generation.

## Decision

Use **Google Gemini 2.0 Flash** as the sole LLM for all chatbot responses (Normal Mode and Highlight Mode).

**Key Specifications:**
- Model: `gemini-2.0-flash-exp` via Google AI Studio or Vertex AI
- SDK: `google-generativeai` (Python)
- Pricing: $0.075 per 1M input tokens, $0.30 per 1M output tokens (as of Dec 2024)
- Target latency: <400ms for first token (streaming)
- Context window: 1M tokens
- Streaming: Native support via `generate_content_stream()`

## Consequences

### Positive

- **Cost-effective**: ~$0.35 per million tokens total cost (cheapest among Tier-1 models)
- **Fastest response time**: Gemini 2.0 Flash achieves <400ms first token latency, critical for real-time user experience
- **Native streaming**: Built-in streaming support eliminates need for custom chunking/buffering
- **Large context window**: 1M tokens allows sending large textbook chunks without truncation
- **Free tier available**: Google AI Studio provides free tier for hackathon development and testing
- **Good citation quality**: Model trained to generate structured responses with source references
- **Simple integration**: Single SDK (`google-generativeai`) with clean Python API
- **Hackathon-friendly**: Fast setup, no approval process, immediate API access

### Negative

- **Vendor lock-in**: Tight coupling to Google's API and pricing structure
- **Quota limits**: Free tier has rate limits (15 requests/minute, 1M tokens/day) - may need upgrade for production
- **Model updates**: Google may deprecate or change model behavior without notice
- **No fine-tuning**: Cannot customize model for domain-specific textbook terminology
- **API stability**: Experimental models (`-exp` suffix) may have breaking changes
- **Limited control**: Cannot adjust temperature, top-p independently (model-specific constraints)
- **Regional availability**: Some regions may have restricted access or higher latency

## Alternatives Considered

### Alternative A: OpenAI GPT-4 Turbo
- **Cost**: ~$10 per 1M input tokens, ~$30 per 1M output tokens (10-100x more expensive)
- **Latency**: ~800ms-1.5s first token (2-4x slower)
- **Why rejected**: Prohibitively expensive for hackathon with free hosting (Railway/Vercel limits), slower response times fail <400ms requirement

### Alternative B: Anthropic Claude 3.5 Sonnet
- **Cost**: ~$3 per 1M input tokens, ~$15 per 1M output tokens (10-50x more expensive than Gemini)
- **Latency**: ~600ms first token (50% slower)
- **Why rejected**: Higher cost, slower latency, and similar vendor lock-in without cost advantage

### Alternative C: Open-source LLMs (Llama 3.1 70B, Mistral Large)
- **Cost**: Free (self-hosted) or ~$0.50-1.00 per 1M tokens (hosted)
- **Latency**: ~1-3s first token (self-hosted on CPU/limited GPU)
- **Why rejected**:
  - Self-hosting requires GPU infrastructure (expensive, complex deployment)
  - Hosted options (Replicate, Together.ai) have similar pricing to Gemini with worse latency
  - Quality for citation generation inferior to Gemini 2.0 Flash
  - Setup complexity conflicts with 12-day hackathon timeline

### Alternative D: Multi-model routing (GPT-4 for complex, Gemini for simple)
- **Approach**: Route queries based on complexity score
- **Why rejected**:
  - Adds architectural complexity (routing logic, fallback handling)
  - Inconsistent user experience (varying response styles)
  - Doubles integration effort (two SDKs, two billing systems)
  - Violates "simplest viable solution" principle

## References

- Feature Spec: `specs/rag-chatbot-integration/spec.md` (FR-005, NFR-001)
- Implementation Plan: `specs/rag-chatbot-integration/plan.md` (pending)
- Related ADRs:
  - ADR-002 (No LangGraph/LangChain - enables direct Gemini SDK usage)
  - ADR-003 (LiteLLM wrapper - provides OpenAI compatibility fallback)
- Google AI Studio Pricing: https://ai.google.dev/pricing
- Gemini 2.0 Flash Benchmarks: https://deepmind.google/technologies/gemini/flash/

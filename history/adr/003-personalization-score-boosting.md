# ADR-003: Personalization via Relevance Score Boosting

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-27
- **Feature:** better-auth-signup-signin
- **Context:** Authenticated users provide their programming experience (Python, C++, None, Both) during signup. The chatbot must personalize content recommendations by prioritizing relevant programming language examples. The system uses Qdrant vector database for semantic search with Cohere embeddings. Performance requirement: chatbot responses must complete within 5 seconds (NFR-007). Three personalization approaches were considered: pre-filtering, LLM re-ranking, and score boosting.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? YES - defines core personalization algorithm
     2) Alternatives: Multiple viable options considered with tradeoffs? YES - pre-filtering, LLM re-ranking, score boosting
     3) Scope: Cross-cutting concern (not an isolated detail)? YES - affects UX, performance, search quality
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Implement personalization using **relevance score boosting** after vector search:

- **Algorithm**: Multiply Qdrant similarity scores by boost factors based on user's programming_experience
- **Boost Factors**:
  - Python users: 1.3x for chunks containing "python", "rclpy", ".py"
  - C++ users: 1.3x for chunks containing "c++", "rclcpp", ".cpp", ".hpp"
  - None (beginners): 1.2x for chunks containing "explanation", "overview", "concept"
  - Both: 1.0x (no boost, balanced results)
- **Implementation**: Apply boost to top-k (k=20) Qdrant results before sorting by score
- **Metadata Dependency**: Use `language` field in Qdrant payload (if missing, treat as language-agnostic)
- **Fallback**: If no boosted results above threshold, return generic top results (graceful degradation)
- **Performance Budget**: <50ms overhead for score boosting logic (acceptable within 5s total response time)

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

- **Minimal Latency**: <50ms overhead vs 2-3s for LLM re-ranking (maintains 5s response time per NFR-007)
- **Preserves Diversity**: "Soft" personalization shows Python first but doesn't hide C++ (users benefit from cross-language concepts)
- **Simple Implementation**: ~30 LOC modification to existing Qdrant query logic (lowest complexity)
- **Graceful Degradation**: Works even if metadata is missing (treats content as language-agnostic)
- **Tunable Parameters**: Boost factors (1.3x, 1.2x) are configurable based on user feedback
- **No Additional API Costs**: No extra Cohere API calls (vs LLM re-ranking which doubles cost)
- **Transparent Logic**: Score boosting is explainable and debuggable (vs black-box LLM re-ranking)
- **Fast Iteration**: Can adjust boost factors without retraining models or rebuilding indexes

<!-- Example: Integrated tooling, excellent DX, fast deploys, strong TypeScript support -->

### Negative

- **Less Aggressive**: "Soft" personalization may not satisfy users expecting strict language filtering
- **Metadata Dependency**: Requires `language` field in Qdrant payloads (may be incomplete for conceptual content)
- **Keyword-Based**: Relies on keyword matching ("python", "c++") not semantic understanding (may miss language-specific concepts without keywords)
- **No ML Sophistication**: Rule-based algorithm doesn't learn from user behavior or implicit feedback
- **Fixed Boost Factors**: 1.3x boost is arbitrary (not data-driven); may need tuning based on user feedback
- **Limited Context**: Doesn't account for user's skill level (beginner Python vs expert Python treated same)

<!-- Example: Vendor lock-in to Vercel, framework coupling, learning curve -->

## Alternatives Considered

### Alternative 1: Pre-Filtering (Metadata Filter Before Vector Search)
- **Components**: Qdrant filter query (`must: {key: "language", match: "python"}`), strict language exclusion
- **Why Rejected**: Too aggressive - Python users miss valuable conceptual explanations in C++ documentation. Reduces result diversity. May return zero results if metadata incomplete. Poor UX when filter too strict.

### Alternative 2: Post-Search LLM Re-Ranking
- **Components**: Retrieve top 50 generic results, send to Cohere with user context ("User prefers Python"), re-rank based on LLM evaluation
- **Why Rejected**: Adds 2-3s latency (violates 5s response time requirement per NFR-007). Doubles API cost (extra Cohere call). Over-engineered for MVP. Black-box logic hard to debug/tune.

### Alternative 3: Hybrid Pre-Filtering + Score Boosting
- **Components**: Apply loose filter (exclude opposite language), then boost within filtered results
- **Why Rejected**: Added complexity (two-stage logic). Pre-filtering still reduces diversity. Score boosting alone achieves balance without filter overhead. Premature optimization.

### Alternative 4: Collaborative Filtering (ML-Based Recommendations)
- **Components**: Track user interactions (clicks, dwell time), train recommendation model, personalize based on similar users
- **Why Rejected**: Requires significant data collection (cold start problem). Complex ML infrastructure. Overkill for simple programming language preference. Implement only if simple boosting insufficient.

### Alternative 5: No Personalization (Generic Results Only)
- **Components**: Return top Qdrant results regardless of user preference, optional auth provides no benefit
- **Why Rejected**: Misses core value proposition (constitution principle: "User experience drives content ranking"). No incentive for users to sign up. Wastes user-provided programming experience data.

<!-- Group alternatives by cluster:
     Alternative Stack A: Remix + styled-components + Cloudflare
     Alternative Stack B: Vite + vanilla CSS + AWS Amplify
     Why rejected: Less integrated, more setup complexity
-->

## References

- Feature Spec: [specs/002-better-auth-signup-signin/spec.md](../../specs/002-better-auth-signup-signin/spec.md#user-story-5---personalized-content-ranking-priority-p2)
- Implementation Plan: [specs/002-better-auth-signup-signin/plan.md](../../specs/002-better-auth-signup-signin/plan.md#decision-3-relevance-score-boosting-not-pre-filtering-or-llm-re-ranking)
- Data Model: [specs/002-better-auth-signup-signin/data-model.md](../../specs/002-better-auth-signup-signin/data-model.md#personalization-data-flow)
- Related ADRs:
  - ADR-001 (JWT tokens embed programming_experience for personalization context)
- Evaluator Evidence: [history/prompts/better-auth-signup-signin/0001-implementation-plan-better-auth-signup-signin.plan.prompt.md](../prompts/better-auth-signup-signin/0001-implementation-plan-better-auth-signup-signin.plan.prompt.md)

## Evaluation Plan

**Success Criteria** (from spec.md SC-002, SC-003):
- Python users receive Python code examples in 80%+ of technical queries
- C++ users receive C++ code examples in 80%+ of technical queries

**Testing Approach**:
1. Create test queries: "How do I create a ROS2 node?" (should return language-specific code)
2. Test with Python user: Verify Python (rclpy) examples appear first in response
3. Test with C++ user: Verify C++ (rclcpp) examples appear first in response
4. Test with None user: Verify conceptual explanations prioritized over code-heavy content
5. Test with Both user: Verify balanced Python/C++ results

**Tuning Strategy**:
- If <80% accuracy: Increase boost factor (e.g., 1.3x → 1.5x)
- If diversity suffers: Decrease boost factor (e.g., 1.3x → 1.1x)
- If keyword matching insufficient: Add more keywords to boost list (e.g., "node.py", "subscriber.cpp")

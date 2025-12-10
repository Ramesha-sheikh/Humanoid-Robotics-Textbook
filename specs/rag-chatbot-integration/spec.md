# Feature Specification: RAG Chatbot Integration

**Feature Branch**: `rag-chatbot-integration`
**Created**: 2025-12-10
**Status**: Draft
**Input**: User description: "Embed a RAG chatbot powered by Gemini 2.0 Flash inside the Physical AI & Humanoid Robotics textbook with support for both full-book and highlight-specific queries"

## Overview

This feature adds an intelligent RAG (Retrieval-Augmented Generation) chatbot to the textbook that allows readers to ask questions about the content. The chatbot operates in two modes: Normal Mode (querying the entire book) and Highlight Mode (querying only user-selected text). The chatbot will be embedded within the published DocOnce book via iframe.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ask Questions About Full Book Content (Priority: P1)

A student reading the textbook wants to quickly find information about a specific topic (e.g., "What are the key components of humanoid robot control systems?") without manually searching through chapters.

**Why this priority**: This is the core functionality that delivers immediate value. Without this, there's no chatbot. This establishes the basic question-answering capability that all other features build upon.

**Independent Test**: Can be fully tested by loading the textbook, opening the chatbot interface, typing a question about any chapter content, and receiving a relevant answer with source citations. Delivers standalone value even without other features.

**Acceptance Scenarios**:

1. **Given** the textbook is loaded with chatbot embedded, **When** user types "What is inverse kinematics?" and sends, **Then** chatbot returns a relevant answer with citations to specific sections
2. **Given** user asks a question about chapter 5, **When** the answer is generated, **Then** the response includes source references (chapter, section, page) from the textbook
3. **Given** user asks a complex question spanning multiple topics, **When** chatbot responds, **Then** answer synthesizes information from multiple relevant sections with proper citations
4. **Given** user asks about a topic not covered in the book, **When** chatbot processes the query, **Then** system responds with "I cannot find information about this in the textbook" rather than hallucinating

---

### User Story 2 - Stream Responses in Real-Time (Priority: P1)

A user asks a complex question and wants to see the answer appear progressively rather than waiting for the complete response.

**Why this priority**: Critical for user experience. Long responses (5-10 seconds) feel unresponsive without streaming. This prevents users from thinking the system has frozen.

**Independent Test**: Can be tested by asking a question that generates a long response (200+ words) and observing that text appears word-by-word or chunk-by-chunk rather than all at once. Works independently of other features.

**Acceptance Scenarios**:

1. **Given** user submits a question requiring a detailed answer, **When** chatbot begins generating response, **Then** user sees text appearing progressively in real-time
2. **Given** streaming is in progress, **When** user wants to interrupt, **Then** system provides a way to stop generation (e.g., stop button)
3. **Given** network connection is slow, **When** streaming response, **Then** system displays partial content as it arrives without waiting for complete response

---

### User Story 3 - Context-Specific Queries on Highlighted Text (Priority: P2)

A reader highlights a specific paragraph about "gait planning" and wants to ask questions specifically about that highlighted section without the chatbot pulling in information from the entire book.

**Why this priority**: This differentiates the chatbot from generic search. It enables focused, contextual learning on specific passages. High value but book-wide search must work first.

**Independent Test**: Can be tested by highlighting any text passage in the textbook, opening chatbot in "Highlight Mode", asking a question about that specific passage, and verifying the answer only references the highlighted content. Provides unique value independent of normal mode.

**Acceptance Scenarios**:

1. **Given** user highlights a paragraph about sensor fusion, **When** user enables Highlight Mode and asks "What sensors are mentioned here?", **Then** chatbot answers using ONLY the highlighted text, not the entire book
2. **Given** user highlights text and asks a question in Highlight Mode, **When** the highlighted section doesn't contain enough information, **Then** chatbot responds "The highlighted section doesn't contain enough information to answer this question"
3. **Given** user switches between Normal Mode and Highlight Mode, **When** asking the same question, **Then** answers differ appropriately based on the context scope

---

### User Story 4 - Persistent Chat History Across Sessions (Priority: P3)

A user returns to the textbook days later and wants to review previous conversations and continue where they left off.

**Why this priority**: Enhances learning continuity but not essential for initial value delivery. Users can still benefit from the chatbot without history. Nice-to-have that improves retention.

**Independent Test**: Can be tested by having a conversation, closing the browser, reopening the textbook later, and verifying the chat history is restored. Independent feature that doesn't block other functionality.

**Acceptance Scenarios**:

1. **Given** user has a conversation with the chatbot, **When** user closes and reopens the textbook, **Then** previous chat history is displayed
2. **Given** user has multiple chat sessions, **When** viewing history, **Then** sessions are organized by date/time with clear separation
3. **Given** user wants to clear history, **When** user clicks "Clear History", **Then** all previous conversations are deleted after confirmation

---

### User Story 5 - Textbook Content Ingestion (Priority: P0 - Pre-requisite)

Before users can interact with the chatbot, the complete textbook content (PDF/HTML) must be processed and stored in the vector database.

**Why this priority**: This is P0 (before P1) because it's a pre-requisite. The chatbot cannot function without indexed content. This is an admin/setup task, not a user-facing feature, but critical infrastructure.

**Independent Test**: Can be tested by running the ingestion pipeline with the textbook PDF/HTML, verifying all chapters are chunked and embedded, checking Qdrant Cloud contains the expected number of vectors, and performing a test query to confirm retrieval works.

**Acceptance Scenarios**:

1. **Given** textbook PDF/HTML files are available, **When** ingestion script runs, **Then** all chapters are chunked, embedded with text-embedding-004, and stored in Qdrant Cloud
2. **Given** ingestion completes, **When** performing a similarity search for "kinematics", **Then** relevant chunks from the textbook are retrieved with scores
3. **Given** textbook is updated, **When** re-running ingestion, **Then** system updates vector database without duplicating existing content

---

### Edge Cases

- **What happens when user asks a question while previous response is still streaming?** System should either queue the new question or cancel the current stream and start new generation.
- **How does system handle very long highlighted text (multiple pages)?** System should either chunk the highlight or warn user that selection is too large for focused query.
- **What happens when Qdrant Cloud is unavailable?** Display user-friendly error: "Search service temporarily unavailable. Please try again."
- **What happens when Gemini API quota is exceeded?** Display error: "Daily usage limit reached. Please try again tomorrow."
- **How does system handle malformed or injection-style queries?** Input validation should sanitize queries and reject potentially malicious patterns.
- **What happens when textbook content contains images or diagrams?** Initial version will ingest only text; images are excluded from embeddings (limitation documented).
- **What happens when user asks questions in a language other than English?** Gemini 2.0 Flash supports multilingual, but if textbook is English-only, clarify language mismatch.
- **What happens when multiple users query simultaneously on free tier?** Qdrant Cloud free tier may have rate limits; implement queueing or backoff strategy.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST ingest complete textbook content (PDF/HTML format) and store embeddings in Qdrant Cloud vector database
- **FR-002**: System MUST generate embeddings using Google's text-embedding-004 model
- **FR-003**: System MUST provide Normal Mode where users can query the entire textbook content
- **FR-004**: System MUST provide Highlight Mode where users can query ONLY their selected/highlighted text
- **FR-005**: System MUST use Gemini 2.0 Flash (via google-generativeai SDK) for response generation
- **FR-006**: System MUST stream responses to the user interface in real-time (not return complete response at once)
- **FR-007**: System MUST include source citations with each answer, referencing specific sections/pages of the textbook
- **FR-008**: System MUST persist chat history in Neon Serverless Postgres database
- **FR-009**: System MUST be embeddable in the textbook via iframe
- **FR-010**: Backend MUST be implemented using FastAPI with Python 3.11
- **FR-011**: Frontend MUST be implemented using Next.js 15 (App Router)
- **FR-012**: System MUST NOT use LangChain or LangGraph (simple retrieval → generation pipeline only)
- **FR-013**: System MUST handle errors gracefully (API failures, network issues, empty results)
- **FR-014**: System MUST validate and sanitize user input to prevent injection attacks
- **FR-015**: System MUST provide visual indicator when switching between Normal and Highlight modes

### Non-Functional Requirements

- **NFR-001**: Response latency SHOULD be under 3 seconds for first token (streaming start)
- **NFR-002**: System SHOULD handle at least 10 concurrent users (free tier constraint)
- **NFR-003**: Embedding generation during ingestion SHOULD complete within 30 minutes for full textbook
- **NFR-004**: Chat history SHOULD be retained for at least 90 days
- **NFR-005**: System SHOULD work on modern browsers (Chrome 90+, Firefox 88+, Safari 14+, Edge 90+)

### Key Entities

- **TextbookChunk**: Represents a segment of textbook content with metadata (chapter, section, page numbers) and vector embedding
- **ChatMessage**: Represents a single message in conversation history with user query, assistant response, timestamp, and mode (Normal/Highlight)
- **ChatSession**: Represents a user's conversation session with unique ID, creation timestamp, and associated messages
- **HighlightContext**: Represents user-selected text with start/end positions and associated metadata for Highlight Mode queries

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can receive relevant answers to questions about textbook content within 3 seconds (first token) for 90% of queries
- **SC-002**: System correctly retrieves and cites relevant textbook sections in 85% of test queries
- **SC-003**: Highlight Mode restricts answers to ONLY selected text in 95% of test cases (no leakage from other chapters)
- **SC-004**: Chat history persists across browser sessions for 100% of logged interactions
- **SC-005**: System handles at least 10 concurrent users without performance degradation
- **SC-006**: Streaming responses begin within 2 seconds and complete incrementally for long answers (200+ words)
- **SC-007**: Textbook ingestion pipeline completes successfully for 100,000+ word textbook within 30 minutes

### User Acceptance Criteria

- User can embed chatbot iframe in published textbook without layout breaks
- User can toggle between Normal and Highlight modes with clear visual feedback
- User sees source citations that link back to relevant textbook sections
- User can read streaming responses smoothly without UI flicker or lag

## Technical Constraints

- **TC-001**: MUST use Qdrant Cloud Free Tier (capacity and rate limits apply)
- **TC-002**: MUST use Neon Serverless Postgres Free Tier for chat history
- **TC-003**: MUST use Gemini 2.0 Flash via Google AI Studio or Vertex AI
- **TC-004**: MUST avoid LangChain/LangGraph dependencies (per project requirement)
- **TC-005**: Embedding model MUST be text-embedding-004 (Google)
- **TC-006**: Backend MUST run on Python 3.11
- **TC-007**: Frontend MUST use Next.js 15 App Router architecture

## Out of Scope

- **OOS-001**: Multi-user collaboration features (shared chats, annotations)
- **OOS-002**: Voice input/output for chatbot interactions
- **OOS-003**: Integration with external knowledge sources beyond the textbook
- **OOS-004**: Advanced analytics dashboard for tracking user queries
- **OOS-005**: Mobile native apps (web-only via responsive iframe)
- **OOS-006**: Support for multiple textbooks or multi-tenant architecture
- **OOS-007**: Image/diagram understanding (text-only RAG for initial version)
- **OOS-008**: Real-time collaborative highlighting between multiple users

## Open Questions

- How should citations be formatted? (Chapter X, Section Y format vs. page numbers vs. hyperlinks)
- What is the maximum highlighted text length for Highlight Mode? (e.g., 2000 characters)
- Should chat history be user-specific (login required) or session-based (anonymous)?
- What chunk size and overlap strategy should be used for textbook ingestion? (e.g., 512 tokens with 50 token overlap)
- Should the system support follow-up questions with conversational context, or treat each query independently?
- What is the strategy for handling textbook updates/revisions after initial ingestion?

## Dependencies

- **DEP-001**: Qdrant Cloud account with API access
- **DEP-002**: Google AI Studio or Vertex AI account with Gemini 2.0 Flash API access
- **DEP-003**: Neon Serverless Postgres database instance
- **DEP-004**: Textbook content in PDF or HTML format
- **DEP-005**: Published textbook platform that supports iframe embedding

## Risks

- **RISK-001**: Gemini API quota may be insufficient for hackathon demo traffic (mitigation: monitor usage, implement rate limiting)
- **RISK-002**: Qdrant Cloud free tier may have undocumented limits (mitigation: test with production-scale data early, have paid upgrade plan ready)
- **RISK-003**: Complex textbook formatting (tables, equations) may not embed well as text (mitigation: document limitations, focus on narrative text)
- **RISK-004**: Streaming may have latency issues over slow networks (mitigation: implement adaptive buffering, show loading indicators)
- **RISK-005**: Cross-origin iframe embedding may face CORS or CSP restrictions (mitigation: configure proper headers, test embedding early)

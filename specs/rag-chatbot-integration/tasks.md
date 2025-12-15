# Tasks Breakdown
**Project:** Integrated RAG Chatbot for Physical AI & Humanoid Robotics Textbook
**Requirement:** Hackathon Req 2 – RAG Chatbot embed in published book (Docusaurus site)
**Key Features Needed:**
- Full book content pe questions
- User-selected text pe contextual questions
- Answers sirf book se (grounded, no hallucination)

## Phase 1: Setup (Project Initialization)
- [ ] T001 Create `rag-backend` directory for FastAPI backend.
- [ ] T002 Initialize Python virtual environment and install dependencies in `rag-backend/venv`.
- [ ] T003 Create `rag-backend/data` directory for crawled data.
- [ ] T004 Create `rag-backend/1_crawl_playwright.py` for crawling sitemap.
- [ ] T005 Create `rag-backend/2_embed_and_upload.py` for embedding and Qdrant upload.
- [ ] T006 Create `rag-backend/main.py` for FastAPI application.

## Phase 2: Foundational (Backend Core Development)
- [ ] T007 [US1] Implement document ingestion pipeline in `rag-backend/document_processor.py`.
- [ ] T008 [US1] Implement RAG query pipeline in `rag-backend/main.py`.
- [ ] T009 [US1] Define `/health`, `/chat`, `/chat/stream` API endpoints in `rag-backend/main.py`.

## Phase 3: User Story 1 (User can ask questions on full book content)
**Goal:** User can ask general questions about the book and get accurate answers.
**Independent Test Criteria:** A user can type a general question into the chatbot, and receive a relevant answer based on the book's content.
- [ ] T010 [US1] Integrate existing `my-website/src/components/ChatBot/` components into Docusaurus frontend.
- [ ] T011 [US1] Modify `my-website/src/components/RagChatbot.tsx` to act as entry point and render chat UI.
- [ ] T012 [US1] Implement `api.ts` client to send normal chat queries to `/chat` endpoint.
- [ ] T013 [US1] Update `ChatMessage.tsx` or `ChatWindow.tsx` to display LLM responses.

## Phase 4: User Story 2 (User can ask contextual questions on selected text)
**Goal:** User can highlight text and ask a specific question based on that context.
**Independent Test Criteria:** A user can highlight a section of text, trigger a contextual query, and receive an answer that is specifically grounded in the selected text.
- [ ] T014 [US2] Implement `highlight_context` capture within Docusaurus pages (`my-website/src/theme/Layout/index.tsx`).
- [ ] T015 [US2] Modify `api.ts` to send `selected_text` with `highlight` mode to FastAPI `/chat` endpoint.
- [ ] T016 [US2] Update frontend to display contextual responses and attributed sources.

## Phase 5: Polish & Cross-Cutting Concerns
- [ ] T017 Manage environment variables (`NEXT_PUBLIC_API_URL`, `COHERE_API_KEY`, `QDRANT_URL`, `QDRANT_API_KEY`).
- [ ] T018 Set up deployment strategy for FastAPI backend (Vercel Serverless Function).
- [ ] T019 Configure Docusaurus deployment on Vercel to point to deployed FastAPI backend via `NEXT_PUBLIC_API_URL`.
- [ ] T020 Verify fast cold start and free tier limits for both frontend and backend.
- [ ] T021 Conduct comprehensive testing for accuracy, grounding, and no hallucinations.
- [ ] T022 Ensure no API keys are exposed in the frontend.

## Dependencies:
- Phase 1 must be completed before Phase 2.
- Phase 2 must be completed before Phase 3.
- Phase 3 must be completed before Phase 4.

## Parallel Execution Examples (within phases):
- **Phase 3:** T010, T011, T012, T013 can be worked on in parallel once the backend endpoints are stable.
- **Phase 4:** T014, T015, T016 can be worked on in parallel.

## Implementation Strategy:
- MVP first, focusing on core functional requirements (User Story 1, then User Story 2).
- Incremental delivery, ensuring each phase is stable and testable before proceeding.
- Prioritize backend development, then frontend integration, and finally deployment considerations.
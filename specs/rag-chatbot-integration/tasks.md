# Tasks: RAG Chatbot Integration

**Input**: Design documents from `specs/rag-chatbot-integration/`
**Prerequisites**: spec.md ✅, ADRs (001-005) ✅
**Timeline**: 12 days (Day 1-12)

**Organization**: Tasks grouped by user story (P0-P3) to enable independent implementation and MVP increments.

## Format: `[ID] [P?] [Day] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Day]**: Suggested implementation day (Days 1-12)
- **[Story]**: User story (P0=Pre-requisite, US1=User Story 1, etc.)

## Path Conventions

- **Backend**: `backend/` (FastAPI + Python 3.11)
- **Frontend**: `frontend/` (Next.js 15 App Router)
- **Docs**: `specs/rag-chatbot-integration/`
- **ADRs**: `history/adr/`

---

## Phase 1: Setup & Infrastructure (Days 1-2) 🚀

**Purpose**: Repository setup, API keys, and basic project structure

**Day 1: Repository & Environment Setup**

- [ ] T001 [Day 1] [P0] Create new GitHub repository `humanoid-robotics-rag-chatbot`
  - **Acceptance**: Repo created with `main` branch, README.md, and .gitignore
  - **Files**: `README.md`, `.gitignore` (Python + Node)

- [ ] T002 [Day 1] [P0] Add specification and ADR files to repository
  - **Acceptance**: All 6 files committed: `spec.md` + 5 ADRs (001-005)
  - **Files**: `specs/rag-chatbot-integration/spec.md`, `history/adr/001-*.md` through `history/adr/005-*.md`

- [ ] T003 [Day 1] [P] [P0] Obtain Gemini API key from Google AI Studio
  - **Acceptance**: API key created, tested with curl request, 15 QPS limit confirmed
  - **Command**: `curl -H "Content-Type: application/json" -d '{"contents":[{"parts":[{"text":"test"}]}]}' "https://generativelanguage.googleapis.com/v1beta/models/gemini-2.0-flash-exp:generateContent?key=YOUR_KEY"`
  - **Validation**: Receive 200 OK response with generated text

- [ ] T004 [Day 1] [P] [P0] Setup Qdrant Cloud free tier account
  - **Acceptance**: Qdrant cluster created, URL and API key obtained, test connection successful
  - **Test**: Connect using `qdrant-client` Python library and list collections (should be empty initially)

- [ ] T005 [Day 1] [P] [P0] Setup Neon Serverless Postgres database
  - **Acceptance**: Neon project created, database URL obtained, test connection successful
  - **Command**: `psql $NEON_DATABASE_URL -c "SELECT version();"`
  - **Validation**: PostgreSQL version printed (15+)

**Day 2: Project Structure & Dependencies**

- [ ] T006 [Day 2] [P] [P0] Initialize backend project structure (FastAPI)
  - **Acceptance**: Backend directory with FastAPI boilerplate, virtual environment, requirements.txt
  - **Files**: `backend/app/main.py`, `backend/requirements.txt`, `backend/.env.example`
  - **Dependencies**: fastapi, uvicorn, python-dotenv, pydantic-settings, google-generativeai, qdrant-client, asyncpg
  - **Test**: Run `uvicorn app.main:app --reload` → see "Hello World" on http://localhost:8000

- [ ] T007 [Day 2] [P] [P0] Initialize frontend project structure (Next.js 15)
  - **Acceptance**: Frontend directory with Next.js App Router, TypeScript, Tailwind CSS
  - **Command**: `npx create-next-app@latest frontend --typescript --tailwind --app --no-src-dir`
  - **Files**: `frontend/app/page.tsx`, `frontend/tailwind.config.ts`
  - **Test**: Run `npm run dev` → see Next.js welcome page on http://localhost:3000

- [ ] T008 [Day 2] [P0] Create .env files with environment variables
  - **Acceptance**: `.env` files created (gitignored), `.env.example` committed with placeholders
  - **Files**:
    - `backend/.env` (GEMINI_API_KEY, QDRANT_URL, QDRANT_API_KEY, NEON_DATABASE_URL, CORS_ORIGINS)
    - `frontend/.env.local` (NEXT_PUBLIC_API_URL)
  - **Test**: Load env vars in Python using `pydantic-settings` and verify all keys present

- [ ] T009 [Day 2] [P0] Create config module with Pydantic settings
  - **Acceptance**: `backend/app/config.py` validates all environment variables on startup
  - **Files**: `backend/app/config.py`
  - **Test**: Run with missing GEMINI_API_KEY → should fail with validation error

**Checkpoint**: Environment and project structure ready ✅

---

## Phase 2: Foundational (Days 2-3) 📚

**Purpose**: Data ingestion pipeline (P0 - Pre-requisite for all user stories)

### User Story P0: Textbook Content Ingestion

**Goal**: Process textbook PDF/HTML → chunks → Qdrant Cloud

**Day 2-3: Ingestion Pipeline**

- [ ] T010 [Day 2] [P0] Create ingestion script skeleton
  - **Acceptance**: `backend/scripts/ingestion.py` with CLI arguments for PDF path
  - **Files**: `backend/scripts/ingestion.py`
  - **Usage**: `python scripts/ingestion.py --pdf path/to/textbook.pdf`

- [ ] T011 [Day 2] [P] [P0] Implement PDF text extraction
  - **Acceptance**: Extract text from PDF preserving structure, detect page breaks
  - **Libraries**: `PyMuPDF` (fitz) or `pypdf`
  - **Output**: List of `(page_num, text)` tuples
  - **Test**: Extract from 5-page sample PDF → verify all text extracted with correct page numbers

- [ ] T012 [Day 3] [P0] Implement chunking strategy with metadata
  - **Acceptance**: Split text into 512-token chunks with 50-token overlap, preserve page/section metadata
  - **Algorithm**: Use `tiktoken` to count tokens, sliding window with overlap
  - **Metadata**: Each chunk has `{chapter, section, page, book_title}`
  - **Test**: Process 10-page chapter → verify chunks have correct page ranges and no text loss

- [ ] T013 [Day 3] [P] [P0] Implement embedding generation with text-embedding-004
  - **Acceptance**: Generate embeddings for chunks using Google's text-embedding-004
  - **API**: Google Generative AI `embed_content()` method
  - **Batch**: Process 100 chunks per API call (rate limit: 1500/min)
  - **Test**: Embed 10 chunks → verify vector dimensions (768 for text-embedding-004)

- [ ] T014 [Day 3] [P0] Upload chunks to Qdrant Cloud
  - **Acceptance**: Create collection, upsert chunks with vectors + metadata
  - **Collection**: `textbook` with vector size 768, distance metric: Cosine
  - **Payload**: Store `{text, chapter, section, page, book_title}` for each chunk
  - **Test**: Upload 100 chunks → query Qdrant for "kinematics" → verify retrieval works

- [ ] T015 [Day 3] [P0] End-to-end ingestion pipeline test
  - **Acceptance**: Run `python scripts/ingestion.py --pdf textbook.pdf` → all chapters ingested
  - **Validation**:
    - Check Qdrant collection size matches expected chunk count
    - Perform 5 test queries → verify relevant chunks retrieved
    - Verify metadata filtering works (filter by page range)
  - **Performance**: Complete within 30 minutes for full textbook (per NFR-003)

**Checkpoint**: Textbook fully ingested in Qdrant Cloud ✅

---

## Phase 3: User Story 1 - Full Book Q&A (Days 4-5) 🎯 MVP

**Goal**: Users can ask questions about entire textbook and receive relevant answers

**Day 4: RAG Function (Normal Mode)**

- [ ] T016 [Day 4] [US1] Implement Qdrant retrieval function (Normal Mode)
  - **Acceptance**: `backend/app/rag.py` → `retrieve_normal_mode(query: str) → List[Chunk]`
  - **Algorithm**: Embed query with text-embedding-004, search Qdrant (top 5 chunks, cosine similarity)
  - **Return**: List of chunks with `{text, chapter, section, page}` metadata
  - **Test**: Query "What is inverse kinematics?" → verify top 5 chunks are relevant

- [ ] T017 [Day 4] [US1] Implement prompt builder with citation instructions
  - **Acceptance**: `build_prompt(query, chunks) → str` formats context + instructions for Gemini
  - **Template**:
    ```
    You are an expert on Humanoid Robotics. Answer based on the textbook excerpts below.

    TEXTBOOK EXCERPTS:
    [Chunk 1] (Chapter: X, Section: Y, Page: Z)
    ...

    QUESTION: {query}

    ANSWER: Provide a clear answer with citations in format [Chapter X, Section Y, Page Z].
    ```
  - **Test**: Build prompt with 3 chunks → verify format includes all metadata

- [ ] T018 [Day 4] [P] [US1] Implement Gemini 2.0 Flash generation (streaming)
  - **Acceptance**: `gemini_generate_stream(prompt) → AsyncGenerator[str]` yields tokens
  - **SDK**: `google.generativeai.GenerativeModel("gemini-2.0-flash-exp").generate_content_stream()`
  - **Performance**: First token < 400ms (per NFR-001)
  - **Test**: Generate response → measure first token latency, verify streaming works

- [ ] T019 [Day 4] [US1] Implement end-to-end RAG function
  - **Acceptance**: `async def rag_query(query: str, mode="normal") → AsyncGenerator[str]`
  - **Flow**: retrieve → build_prompt → gemini_stream
  - **Test**: Query "What are the key components of humanoid robot control systems?" → verify relevant answer with citations

**Day 5: FastAPI Endpoints**

- [ ] T020 [Day 5] [US1] Implement `/chat` endpoint (non-streaming)
  - **Acceptance**: POST `/chat` with `{query, mode}` → returns complete answer
  - **Response**: `{answer: str, sources: [{chapter, section, page}], latency_ms: int}`
  - **Test**: curl POST with query → verify 200 OK, valid JSON response with citations

- [ ] T021 [Day 5] [US1] Implement `/chat/stream` endpoint (Server-Sent Events)
  - **Acceptance**: POST `/chat/stream` → streams answer tokens via SSE
  - **Format**: `data: {token: "...", done: false}\n\n` for each token, `done: true` on completion
  - **Test**: curl with SSE client → verify streaming tokens arrive progressively

- [ ] T022 [Day 5] [P] [US1] Add CORS middleware for frontend
  - **Acceptance**: Configure CORS to allow requests from localhost:3000 (dev) and production domain
  - **Config**: Use `CORS_ORIGINS` from environment variables
  - **Test**: Make fetch request from frontend → verify no CORS errors

- [ ] T023 [Day 5] [US1] Add error handling and validation
  - **Acceptance**: Handle empty queries, Qdrant failures, Gemini API errors gracefully
  - **Responses**: 400 for invalid input, 503 for service unavailable, 500 for internal errors
  - **Test**: Send invalid request → verify proper error response

**Day 5: User Acceptance Testing (US1)**

- [ ] T024 [Day 5] [US1] Test full-book Q&A with 10 sample questions
  - **Acceptance**: All 10 questions return relevant answers with citations within 3 seconds
  - **Sample Questions**: "What is inverse kinematics?", "How do bipedal robots maintain balance?", etc.
  - **Validation**: Verify citations match retrieved chunks, no hallucinations

**Checkpoint**: User Story 1 (P1) complete - users can ask questions about full textbook ✅

---

## Phase 4: User Story 2 - Real-Time Streaming (Day 6) ⚡

**Goal**: Users see answers appear progressively in real-time

**Day 6: Streaming Response**

- [ ] T025 [Day 6] [US2] Verify streaming already works (from T021)
  - **Acceptance**: `/chat/stream` endpoint streams tokens via SSE
  - **Performance**: First token < 400ms, tokens arrive progressively
  - **Test**: Query with 200+ word answer → verify streaming behavior

- [ ] T026 [Day 6] [US2] Add stop generation endpoint
  - **Acceptance**: POST `/chat/stop` with `{session_id}` cancels active streaming
  - **Implementation**: Use asyncio task cancellation
  - **Test**: Start streaming → call `/chat/stop` mid-stream → verify generation stops

**Checkpoint**: User Story 2 (P1) complete - streaming works ✅

---

## Phase 5: User Story 3 - Highlight Mode (Day 5-6) 🎯

**Goal**: Users can query ONLY highlighted text with zero leakage

**Day 5: Highlight Mode Retrieval**

- [ ] T027 [Day 5] [US3] Define HighlightContext data model
  - **Acceptance**: Pydantic model with `{text, startPage, endPage, chapterSlug}`
  - **Files**: `backend/app/models.py`
  - **Validation**: Ensure startPage <= endPage, chapterSlug is non-empty

- [ ] T028 [Day 5] [US3] Implement metadata-filtered retrieval (Highlight Mode)
  - **Acceptance**: `retrieve_highlight_mode(query, highlight_context) → List[Chunk]`
  - **Algorithm**: Filter Qdrant by `page >= startPage AND page <= endPage AND chapter == chapterSlug`
  - **Limit**: Top 3 chunks from filtered set only
  - **Test**: Query with page range [42-45] → verify no chunks outside range returned

- [ ] T029 [Day 5] [US3] Combine raw highlight text with retrieved chunks
  - **Acceptance**: Prompt includes "SELECTED TEXT:" + raw highlight + "RELATED CONTEXT:" + filtered chunks
  - **Zero-leakage guarantee**: No context from outside page range
  - **Test**: Verify prompt contains only highlight text + filtered chunks

**Day 6: Highlight Mode Endpoint**

- [ ] T030 [Day 6] [US3] Update `/chat` and `/chat/stream` to support mode parameter
  - **Acceptance**: Accept `{query, mode: "normal" | "highlight", highlight_context?: HighlightContext}`
  - **Routing**: If mode == "highlight", use `retrieve_highlight_mode()`, else use `retrieve_normal_mode()`
  - **Test**: Send query with mode="highlight" and page range → verify filtered retrieval

- [ ] T031 [Day 6] [US3] Add visual mode indicator in response
  - **Acceptance**: Response includes `{mode: "normal" | "highlight"}` field
  - **Test**: Query in highlight mode → verify response has mode="highlight"

**Day 6: Zero-Leakage Testing**

- [ ] T032 [Day 6] [US3] Test zero-leakage guarantee with 20 queries
  - **Acceptance**: Highlight 1-page section, ask questions → verify no answers reference other chapters
  - **Test Cases**:
    - Highlight page 42 (kinematics chapter) → ask "What is mentioned about dynamics?" → should say "not in selected text"
    - Highlight 3 pages → ask cross-page question → should only use those 3 pages
  - **Validation**: Manual review + automated check that no source citations outside page range

**Checkpoint**: User Story 3 (P2) complete - highlight mode works with zero leakage ✅

---

## Phase 6: User Story 4 - Chat History (Day 7) 💾

**Goal**: Users can see previous conversations across sessions

**Day 7: Database Schema & Chat History**

- [ ] T033 [Day 7] [US4] Create Neon Postgres schema for chat history
  - **Acceptance**: Tables `chat_sessions` and `chat_messages` created
  - **Schema**:
    ```sql
    CREATE TABLE chat_sessions (
      id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
      created_at TIMESTAMP DEFAULT NOW()
    );

    CREATE TABLE chat_messages (
      id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
      session_id UUID REFERENCES chat_sessions(id),
      role VARCHAR(10) CHECK (role IN ('user', 'assistant')),
      content TEXT,
      mode VARCHAR(10) CHECK (mode IN ('normal', 'highlight')),
      sources JSONB,
      created_at TIMESTAMP DEFAULT NOW()
    );

    CREATE INDEX idx_messages_session ON chat_messages(session_id, created_at);
    ```
  - **Test**: Run migration → verify tables created

- [ ] T034 [Day 7] [US4] Implement chat history save function
  - **Acceptance**: `save_message(session_id, role, content, mode, sources) → UUID`
  - **Implementation**: Use `asyncpg` to insert into `chat_messages`
  - **Test**: Save user query + assistant response → verify rows in database

- [ ] T035 [Day 7] [US4] Implement chat history load function
  - **Acceptance**: `load_session_history(session_id) → List[Message]`
  - **Return**: Ordered list of messages with `{id, role, content, mode, sources, created_at}`
  - **Test**: Load session with 10 messages → verify correct order and content

- [ ] T036 [Day 7] [US4] Add `/sessions` endpoint (list sessions)
  - **Acceptance**: GET `/sessions` → returns list of sessions with `{id, created_at, message_count}`
  - **Test**: Create 3 sessions → call `/sessions` → verify all 3 returned

- [ ] T037 [Day 7] [US4] Add `/sessions/{id}/messages` endpoint (load history)
  - **Acceptance**: GET `/sessions/{id}/messages` → returns messages for session
  - **Test**: Load session history → verify messages in chronological order

- [ ] T038 [Day 7] [P] [US4] Update `/chat` endpoints to save history
  - **Acceptance**: After generating response, save user query + assistant response to database
  - **Session ID**: Accept optional `session_id` in request, create new session if missing
  - **Test**: Send 5 queries in same session → verify all saved in database

- [ ] T039 [Day 7] [US4] Add `/sessions/{id}` DELETE endpoint (clear history)
  - **Acceptance**: DELETE `/sessions/{id}` → deletes session and all messages
  - **Confirmation**: Require confirmation parameter to prevent accidental deletion
  - **Test**: Delete session → verify removed from database

**Checkpoint**: User Story 4 (P3) complete - chat history persists ✅

---

## Phase 7: User Story 6 - Mobile Responsive UI (Days 8-10) 📱

**Goal**: Beautiful chat UI that works on mobile and desktop

**Day 8: Frontend Chat Interface**

- [ ] T040 [Day 8] [US6] Create chat UI layout (desktop)
  - **Acceptance**: `frontend/app/chat/page.tsx` with message list + input area
  - **Components**: ChatWindow, ChatMessage, ChatInput
  - **Styling**: Tailwind CSS with clean, modern design
  - **Test**: Load page → see empty chat with input box

- [ ] T041 [Day 8] [US6] Implement message rendering
  - **Acceptance**: Display user and assistant messages with distinct styling
  - **Features**: User messages (right-aligned, blue), assistant messages (left-aligned, gray), loading indicator
  - **Test**: Render 5 sample messages → verify correct styling

- [ ] T042 [Day 8] [US6] Implement chat input with send button
  - **Acceptance**: Text input + send button, Enter key submits, disable while loading
  - **UX**: Clear input after send, focus returns to input
  - **Test**: Type message → press Enter → verify sent

- [ ] T043 [Day 8] [P] [US6] Implement API client for chat endpoints
  - **Acceptance**: `frontend/lib/api.ts` with `sendMessage(query, mode)` function
  - **Streaming**: Use EventSource or fetch with ReadableStream for SSE
  - **Error handling**: Display error messages to user
  - **Test**: Send message → verify response received

- [ ] T044 [Day 8] [US6] Connect chat UI to streaming endpoint
  - **Acceptance**: Typing message → see streaming response appear token-by-token
  - **UX**: Show "Typing..." indicator while waiting for first token
  - **Test**: Send query → verify streaming animation works

- [ ] T045 [Day 8] [US6] Implement source citations display
  - **Acceptance**: Show source chips below each assistant message
  - **Format**: "Chapter 5, Section 5.2, Page 42" as clickable chips
  - **Styling**: Small, rounded, with hover effect
  - **Test**: Receive message with 3 sources → verify all 3 displayed

**Day 8: Text Selection & Highlight Mode**

- [ ] T046 [Day 8] [US6] Implement text selection detection
  - **Acceptance**: Detect when user selects text using `window.getSelection()`
  - **Trigger**: Show "Ask about selection" button on text selection
  - **Test**: Select paragraph → verify button appears

- [ ] T047 [Day 8] [US6] Extract highlight metadata from selection
  - **Acceptance**: Extract `{text, startPage, endPage, chapterSlug}` from selection
  - **Implementation**: Read page numbers from DOM attributes (e.g., `data-page="42"`)
  - **Test**: Select text spanning pages 42-45 → verify correct page range extracted

- [ ] T048 [Day 8] [US6] Implement mode toggle UI
  - **Acceptance**: Toggle switch or button to switch between Normal and Highlight modes
  - **Visual**: Clear indicator showing current mode (e.g., badge)
  - **Test**: Click toggle → verify mode switches, visual indicator updates

- [ ] T049 [Day 8] [US6] Send highlight context with query
  - **Acceptance**: When in Highlight Mode, include `highlight_context` in API request
  - **Clear indication**: Show "Asking about: [first 50 chars]..." in UI
  - **Test**: Select text → ask question → verify highlight context sent to backend

**Day 9: Streaming UI & Source Citations**

- [ ] T050 [Day 9] [US2] [US6] Implement smooth streaming animation
  - **Acceptance**: Tokens appear smoothly without flicker, auto-scroll to latest message
  - **Performance**: 60 FPS animation, no layout shifts
  - **Test**: Receive 200-word streaming response → verify smooth rendering

- [ ] T051 [Day 9] [US6] Implement source citation click handler
  - **Acceptance**: Clicking source chip scrolls to that section in textbook (if on same page)
  - **Fallback**: Show chapter/section/page in tooltip if cannot scroll
  - **Test**: Click citation → verify scroll or tooltip

- [ ] T052 [Day 9] [P] [US6] Add copy button for assistant messages
  - **Acceptance**: Hover over message → show copy button → click to copy to clipboard
  - **Feedback**: Show "Copied!" toast for 2 seconds
  - **Test**: Copy message → paste in editor → verify text copied

**Day 9-10: Mobile Responsive Design**

- [ ] T053 [Day 9] [US6] Implement mobile layout (viewport < 768px)
  - **Acceptance**: Full-width chat, larger touch targets (44x44px min), readable fonts (16px min)
  - **Responsive**: Stack messages vertically, compact header
  - **Test**: Open on iPhone (375px width) → verify layout works

- [ ] T054 [Day 9] [US6] Fix mobile keyboard behavior
  - **Acceptance**: Input remains visible when keyboard appears, no hidden content
  - **Implementation**: Use `viewport` height units, scroll input into view
  - **Test**: Open on iOS Safari → type → verify input visible above keyboard

- [ ] T055 [Day 9] [US6] Implement mobile text selection for Highlight Mode
  - **Acceptance**: Touch selection works on mobile, "Ask about selection" button is tappable
  - **Touch targets**: Minimum 44x44px per iOS guidelines
  - **Test**: Select text on mobile → verify button appears and is easy to tap

- [ ] T056 [Day 9] [US6] Test mobile device rotation
  - **Acceptance**: Layout adapts smoothly when rotating device, no UI breaks
  - **State preservation**: Chat history remains visible after rotation
  - **Test**: Rotate device during chat → verify no errors, layout adapts

- [ ] T057 [Day 10] [US6] Test on multiple mobile devices
  - **Devices**: iPhone (375px), Android (360px), iPad (768px)
  - **Browsers**: Chrome, Safari, Firefox
  - **Acceptance**: All core features work on all devices (query, streaming, highlight mode)

**Day 10: Dark Mode & Polish**

- [ ] T058 [Day 10] [US6] Implement dark mode toggle
  - **Acceptance**: Toggle between light and dark themes, preference saved in localStorage
  - **Styling**: Use Tailwind's `dark:` classes, smooth transition
  - **Test**: Toggle dark mode → verify all components adapt

- [ ] T059 [Day 10] [US6] Add loading states and skeletons
  - **Acceptance**: Show skeleton loaders while waiting for first token
  - **UX**: Pulsing animation for loading state
  - **Test**: Send query → verify loading state before first token

- [ ] T060 [Day 10] [P] [US6] Add empty state messaging
  - **Acceptance**: Show welcome message with example questions when chat is empty
  - **Examples**: "Try asking: What is inverse kinematics?"
  - **Test**: Load empty chat → verify welcome message

- [ ] T061 [Day 10] [US6] Implement error toast notifications
  - **Acceptance**: Show toast for API errors, timeout errors, and network errors
  - **Auto-dismiss**: Disappear after 5 seconds or on user dismiss
  - **Test**: Trigger error (disconnect network) → verify toast appears

**Checkpoint**: User Story 6 (P1) complete - mobile-responsive UI works ✅

---

## Phase 8: Deployment (Day 11) 🚀

**Goal**: Deploy backend to Railway and frontend to Vercel

**Day 11: Backend Deployment (Railway)**

- [ ] T062 [Day 11] [Deploy] Create Railway project and connect GitHub repo
  - **Acceptance**: Railway project created, linked to `backend/` directory
  - **Auto-deploy**: Push to `main` triggers deployment
  - **Test**: Push commit → verify Railway builds and deploys

- [ ] T063 [Day 11] [Deploy] Configure environment variables in Railway
  - **Acceptance**: Set GEMINI_API_KEY, QDRANT_URL, QDRANT_API_KEY, NEON_DATABASE_URL, CORS_ORIGINS
  - **Production CORS**: Include Vercel production domain
  - **Test**: Check Railway dashboard → verify all vars set

- [ ] T064 [Day 11] [Deploy] Configure Railway service (port, health check)
  - **Acceptance**: Expose port 8000, add health check endpoint GET `/health`
  - **Health check**: Returns `{status: "ok", version: "1.0.0"}`
  - **Test**: Call `/health` → verify 200 OK

- [ ] T065 [Day 11] [Deploy] Test backend deployment
  - **Acceptance**: Call production API → verify `/chat` endpoint works
  - **URL**: `https://your-app.railway.app/chat`
  - **Test**: curl POST with test query → verify response

**Day 11: Frontend Deployment (Vercel)**

- [ ] T066 [Day 11] [Deploy] Create Vercel project and connect GitHub repo
  - **Acceptance**: Vercel project created, linked to `frontend/` directory
  - **Auto-deploy**: Push to `main` triggers deployment
  - **Test**: Push commit → verify Vercel builds and deploys

- [ ] T067 [Day 11] [Deploy] Configure environment variables in Vercel
  - **Acceptance**: Set NEXT_PUBLIC_API_URL to Railway production URL
  - **Test**: Check Vercel dashboard → verify var set

- [ ] T068 [Day 11] [Deploy] Test frontend deployment
  - **Acceptance**: Open production URL → chat works end-to-end
  - **URL**: `https://your-app.vercel.app`
  - **Test**: Send query → verify streaming response from Railway backend

- [ ] T069 [Day 11] [Deploy] Test mobile deployment on real devices
  - **Acceptance**: Open production URL on iPhone and Android → verify all features work
  - **Test**: Query in Normal and Highlight modes → verify responses

**Checkpoint**: Backend and frontend deployed to production ✅

---

## Phase 9: Integration & Demo (Day 12) 🎬

**Goal**: Embed chatbot in textbook and record demo video

**Day 12: Embed in Textbook**

- [ ] T070 [Day 12] [Integration] Create iframe embed code for textbook
  - **Acceptance**: HTML snippet with `<iframe src="https://your-app.vercel.app/chat" />`
  - **Styling**: Responsive iframe with min-height 500px, border-radius
  - **Test**: Embed in test HTML page → verify chatbot loads

- [ ] T071 [Day 12] [Integration] Add iframe to published DocOnce textbook
  - **Acceptance**: Chatbot iframe appears in textbook sidebar or bottom corner
  - **Placement**: Sticky position, does not block content
  - **Test**: Open textbook → verify chatbot accessible on all pages

- [ ] T072 [Day 12] [Integration] Test textbook text selection → chatbot integration
  - **Acceptance**: Select text in textbook → chatbot detects selection for Highlight Mode
  - **Cross-origin**: Ensure text selection works across iframe boundary (may need postMessage)
  - **Test**: Select paragraph → ask question → verify Highlight Mode works

**Day 12: Demo Video**

- [ ] T073 [Day 12] [Demo] Prepare demo script (90 seconds)
  - **Acceptance**: Script covers: (1) Normal Mode query, (2) Highlight Mode query, (3) Streaming demo, (4) Mobile demo
  - **Timing**: 20s intro, 30s Normal Mode, 30s Highlight Mode, 10s mobile

- [ ] T074 [Day 12] [Demo] Record demo video
  - **Acceptance**: 90-second screen recording showing all key features
  - **Features shown**:
    - Ask question about full textbook → see streaming answer with citations
    - Highlight paragraph → ask question → see Highlight Mode response (zero leakage)
    - Show mobile view → demonstrate responsive UI
  - **Tools**: OBS Studio, Loom, or QuickTime
  - **Test**: Watch video → verify all features clearly demonstrated

- [ ] T075 [Day 12] [Demo] Submit hackathon entry
  - **Acceptance**: GitHub repo URL + demo video + live textbook link submitted
  - **Checklist**:
    - README with setup instructions
    - All 5 ADRs committed
    - Live demo URL working
    - Video uploaded (YouTube, Loom, or direct)
  - **Test**: Click all links → verify everything works

**Checkpoint**: Demo recorded and submitted ✅ 🎉

---

## Summary: Task Dependency Graph

```
Day 1-2: Setup (T001-T009)
           ↓
Day 2-3: Ingestion (T010-T015) [P0 - Prerequisite]
           ↓
Day 4-5: User Story 1 - Full Book Q&A (T016-T024) [P1 - MVP]
           ↓
Day 6: User Story 2 - Streaming (T025-T026) [P1]
       User Story 3 - Highlight Mode (T027-T032) [P2]
           ↓
Day 7: User Story 4 - Chat History (T033-T039) [P3]
           ↓
Day 8-10: User Story 6 - Mobile UI (T040-T061) [P1]
           ↓
Day 11: Deployment (T062-T069)
           ↓
Day 12: Integration & Demo (T070-T075)
```

**Total Tasks**: 75 testable tasks across 12 days

**Parallel Opportunities**: T003-T005, T006-T007, T013, T022, T026, T043, T052, T058, T060

**Critical Path**: Setup → Ingestion → RAG Function → Streaming → Frontend UI → Deployment → Demo

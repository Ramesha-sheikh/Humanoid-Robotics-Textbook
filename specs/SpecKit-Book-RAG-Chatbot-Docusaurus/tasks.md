## Phase 1: Project Setup & Boilerplate (1 hour)
1. [ ] Create main folder → `spec-kit-rag-chatbot/`
2. [ ] Initialize Git repo + add proper .gitignore
3. [ ] Create folder structure exactly as specified
4. [ ] Create `docker-compose.yml` (Qdrant + backend services)
5. [ ] Create `.env.example` with all required variables
6. [ ] Create `backend/requirements.txt` with exact packages

## Phase 2: Document Ingestion Pipeline (2–3 hours)
7. [ ] Write `backend/app/ingestion/loader.py` → pure Python MDX loader + chunker (600 tokens, metadata with exact Docusaurus URL)
8. [ ] Write `backend/app/embedding/cohere_embedder.py` → async Cohere embedding function
9. [ ] Write `backend/app/vector/qdrant_client.py` → collection creation, upsert, and search
10. [ ] Write `backend/app/ingestion/ingest.py` → complete one-click ingestion script

## Phase 3: Retrieval Tool & Agent (2 hours)
11. [ ] Write `backend/app/agents/tools.py` → `@tool retrieve_relevant_chunks(query: str) -> str`
12. [ ] Write `backend/app/agents/rag_agent.py` → full Agent with Groq primary + fallback + perfect system prompt
13. [ ] Write `backend/app/config.py` → Pydantic Settings management

## Phase 4: FastAPI Backend (1.5 hours)
14. [ ] Write `backend/app/main.py` → FastAPI app with CORS + health check
15. [ ] Implement POST `/api/chat` → streaming response using agent.run_stream()
16. [ ] Implement POST `/api/ingest` → protected re-ingestion endpoint
17. [ ] Write `backend/Dockerfile`

## Phase 5: Docusaurus Chat Plugin (2 hours)
18. [ ] Create `docusaurus-chat-plugin/src/components/BookChatBot/`
19. [ ] Build floating chat bubble (bottom-right)
20. [ ] Build chat window with streaming messages + source chips
21. [ ] Implement streaming via EventSource to `/api/chat`
22. [ ] Add plugin to your live Docusaurus site (docusaurus.config.js)

## Phase 6: Testing & Deployment (2 hours)
23. [ ] Run full local test: `docker-compose up → python ingest.py → test chat`
24. [ ] Verify Roman Urdu + English answers with correct clickable source links
25. [ ] Deploy backend + Qdrant to Railway or Render
26. [ ] Update frontend to use live backend URL
27. [ ] Final end-to-end test with hackathon-style questions

## Phase 7: Final Polish & Submission
28. [ ] Write complete `README.md` with setup + demo instructions
29. [ ] Record 30–60 second demo video
30. [ ] Prepare live URL + submission ready

**ALL TASKS 100% COMPLETE & ORDERED**
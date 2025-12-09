<!--
Sync Impact Report:
Version Change: 1.0.0 -> 1.0.1
Modified Principles: All principles have been updated according to the new constitution.
Added Sections: Project Goal, Final Tech Stack, Exact Folder Structure, Success Criteria for Hackathon Judges.
Removed Sections: None.

Templates Requiring Updates:
- .specify/templates/plan-template.md: ✅ updated
- .specify/templates/spec-template.md: ✅ updated
- .specify/templates/tasks-template.md: ✅ updated
- .specify/templates/commands/sp.constitution.md: ✅ updated
- README.md: ✅ updated

Follow-up TODOs: None.
-->
# SpecKit-Book-RAG-Chatbot-Docusaurus Constitution

## 1. Project Goal
Production-ready RAG chatbot jo meri Docusaurus website (live) par embed ho aur meri poori AI book (/docs folder ke saare .mdx files) se Roman Urdu + English mix mein perfect jawab de – 5-8 seconds mein + source link ke saath.

## 2. Final Tech Stack (No LangChain, No Next.js)
- Frontend → Docusaurus 3 (already deployed)
- Chat UI → Custom React component (floating bubble)
- Backend → FastAPI (async) + Uvicorn
- Agents → OpenAI Agents SDK v0.28+ (pure, official)
- LLM → Groq (llama-3.1-70b-instant primary) → fallback gpt-4o-mini
- Embeddings → Cohere Free Tier (embed-multilingual-v3.0)
- Vector DB → Qdrant
- Document Loading → Pure Python (os.walk + yaml frontmatter + markdown)
- Deployment → Frontend (existing Vercel/Netlify) | Backend+Qdrant (Railway/Render)

## 3. Exact Folder Structure (Copy-Paste Ready)
spec-kit-rag-chatbot/
├── backend/
│   ├── app/
│   │   ├── main.py                      # FastAPI app
│   │   ├── config.py                    # env + settings (Pydantic Settings)
│   │   ├── agents/
│   │   │   ├── rag_agent.py             # OpenAI Agent definition
│   │   │   └── tools.py                 # retrieval tool
│   │   ├── embedding/
│   │   │   └── cohere_embedder.py       # async embed function
│   │   ├── ingestion/
│   │   │   ├── loader.py                # pure Python MDX loader + chunker
│   │   │   └── ingest.py                # one-click ingestion script
│   │   ├── vector/
│   │   │   └── qdrant_client.py         # collection create + upsert + search
│   │   └── models/
│   │       └── schemas.py               # Pydantic request/response models
│   ├── Dockerfile
│   ├── requirements.txt
│   └── .env.example
├── docusaurus-chat-plugin/
│   └── src/
│       └── components/
│           └── BookChatBot/
│               ├── ChatBubble.tsx
│               ├── ChatMessage.tsx
│               ├── ChatInput.tsx
│               └── api.ts                   # calls backend /chat
├── docker-compose.yml
├── SPECIFICATION.md (this file)
└── README.md

## Core Principles & Constraints:
1.  Source of truth: Only the .mdx files inside /docs folder of my current Docusaurus site
2.  Embeddings: Cohere Free Tier (multilingual model – supports English + Urdu romanized)
3.  Vector Database: Qdrant running locally in Docker (later optional cloud)
4.  Backend Framework: FastAPI (async) + OpenAI Agents SDK (latest version)
5.  LLM: Groq (llama-3.1-70b-instant primary) → fallback gpt-4o-mini
6.  Frontend: Pure Docusaurus 3 – NO Next.js
7.  Chat UI: Embedded React component inside Docusaurus (right sidebar or floating bubble)
8.  Code Style: Clean Python OOP, Pydantic v2 models, proper typing, async/await everywhere
9.  Full SpecKit Plus workflow must be followed – no shortcuts
10. Zero vibe coding – every step documented

## Success Criteria for Hackathon Judges:
- Open my live Docusaurus site
- Ask any complex question from the book (e.g., “Chapter 8 mein ReAct loop kaise implement karte hain?”)
- Chatbot replies correctly within 5–8 seconds WITH source chapter + exact heading/link

## Tech Stack (FINAL):
├── Frontend: Docusaurus 3 (already deployed)
├── Chat Component: React + Tailwind inside Docusaurus plugin/custom component
├── Backend: FastAPI + Uvicorn
├── Agents: OpenAI Agents SDK v0.28+
├── Embeddings: Cohere (free trial key)
├── Vector Store: Qdrant
├── Loader: LangChain DirectoryLoader + MDX support
└── Deployment: Frontend → Vercel/Netlify (existing), Backend → Railway or Render

## Governance
This constitution overrides all other instructions in the repository. Amendments require documentation, approval, and a migration plan. All Pull Requests and reviews must verify compliance with these principles.

**Version**: 1.0.1 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-09

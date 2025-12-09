# /sp.specify — Complete & Final Project Specification
**Project:** SpecKit-Book-RAG-Chatbot-Docusaurus
**Locked Date:** 09 December 2025
**Status:** READY FOR /sp.clarify → /sp.plan

## 1. Project Goal
Production-ready RAG chatbot jo meri Docusaurus website (live) par embed ho aur meri poori AI book (/docs folder ke saare .mdx files) se Roman Urdu + English mix mein perfect jawab de – 5-8 seconds mein + source link ke saath.

## 2. Final Tech Stack (No LangChain, No Next.js)
- Frontend → Docusaurus 3 (already deployed)
- Chat UI → Custom React component (floating bubble)
- Backend → FastAPI (async) + Uvicorn
- Agents → OpenAI Agents SDK v0.28+ (pure, official)
- LLM → Groq (llama-3.1-70b-instant primary) → fallback gpt-4o-mini
- Embeddings → Cohere Free Tier (embed-multilingual-v3.0)
- Vector DB → Qdrant (Docker)
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

## 4. Clarified Details

### 4.1 MDX Files Path
The `.mdx` files containing the book content are located directly in the `/docs` folder at the project root.

### 4.2 Chat Endpoint API Contract (`/chat`)
- **Method:** POST
- **Request Body:** JSON
  ```json
  {
    "user_query": "string",
    "session_id": "string", // Optional, for maintaining conversation history
    "user_id": "string"     // Optional, for user-specific context or analytics
  }
  ```
- **Response Body:** JSON
  ```json
  {
    "answer": "string",
    "sources": [
      {
        "title": "string",
        "url": "string",
        "heading": "string" // Specific heading within the source
      }
    ],
    "session_id": "string" // Returned if provided in request
  }
  ```
- **Authentication:** To be determined during planning, but consider API key or token-based authentication.

### 4.3 Document Chunking Strategy
- **Approach:** Semantic Chunking with inclusion of all content.
- **Chunk Size:** To be determined during planning (e.g., 500-1000 characters)
- **Overlap:** To be determined during planning (e.g., 10-20% of chunk size)
- **Content Handling:**
    - Code blocks, images (with alt text), and other non-text content will be included in chunks.
    - Markdown headings and subheadings will be used as boundaries or metadata to preserve semantic context during chunking.

# Implementation Plan: SpecKit-Book-RAG-Chatbot-Docusaurus

**Branch**: `spec-kit-rag-chatbot` | **Date**: 2025-12-09 | **Spec**: /specs/SpecKit-Book-RAG-Chatbot-Docusaurus/spec.md
**Input**: Feature specification from `/specs/spec-kit-rag-chatbot/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Production-ready RAG chatbot embedded on a Docusaurus website, capable of intelligently answering questions from the AI book (MDX files in `/docs`) in a natural mix of English + Roman Urdu, within 5-8 seconds, with source links. The technical approach involves a FastAPI backend with OpenAI Agents SDK, Cohere embeddings, Qdrant vector database, and a custom React chat UI component within Docusaurus.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: FastAPI, Uvicorn, OpenAI Agents SDK v0.28+, Cohere (embeddings), Qdrant (vector DB)
**Storage**: Qdrant (Docker)
**Testing**: NEEDS CLARIFICATION
**Target Platform**: Linux server
**Project Type**: Web application (Frontend + Backend)
**Performance Goals**: Chatbot replies correctly within 5–8 seconds
**Constraints**: Pure Docusaurus 3 (NO Next.js), No LangChain, OpenAI Agents SDK v0.28+, Cohere Free Tier, Qdrant (Docker)
**Scale/Scope**: Entire AI book in `/docs` folder, Roman Urdu + English mix, production-ready.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- ✅ **Embodied Intelligence First**: Aligned (indirectly, as the book content is about Physical AI).
- ✅ **One Element Per Type Rule**: Not directly applicable to chatbot implementation, but chatbot will process content following this rule.
- ✅ **Professional Learner Experience**: Aligned (chatbot aims for professional experience with accurate, sourced answers).
- ✅ **95%+ Consistency Target**: Aligned (chatbot will aim for consistent responses and source attribution).
- ✅ **Spec-Driven Development**: Aligned (current process adheres to this).
- ✅ **Technology Stack (Locked)**: Aligned (uses Docusaurus, FastAPI, Python Agents, OpenAI/Groq, Cohere, Qdrant as specified).

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)
```text
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
```

**Structure Decision**: The project will utilize a monorepo-like structure with a clear separation between the backend (FastAPI) and the Docusaurus chat plugin (React). This aligns with the provided exact folder structure in the specification, optimizing for maintainability and independent deployment of frontend and backend components.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |

<!--
Sync Impact Report:
Version Change: 1.0.1 -> 2.0.0
Modified Principles: All principles have been updated according to the new constitution.
Added Sections: Project Name, Core Principles, Key Standards, Constraints, Success Criteria.
Removed Sections: Project Goal, Final Tech Stack, Exact Folder Structure, Core Principles & Constraints (old), Success Criteria for Hackathon Judges, Tech Stack (FINAL).

Templates Requiring Updates:
- .specify/templates/plan-template.md: ✅ reviewed
- .specify/templates/spec-template.md: ✅ reviewed
- .specify/templates/tasks-template.md: ✅ reviewed
- .specify/templates/commands/sp.constitution.md: ⚠ pending (file not found)

Follow-up TODOs: None.
-->
# Project Constitution

**Project Name:** Physical AI & Humanoid Robotics Textbook RAG Chatbot Integration

**Core Principles:**
- Accuracy: All answers must be grounded in the book's content only. No hallucinations.
- Privacy & Security: Never expose API keys in frontend. Use environment variables.
- Reproducibility: All code must be runnable with provided keys and free-tier services.
- Clarity: Code must be well-commented, beginner-friendly (for GIAIC students).
- Rigor: Use modern best practices – async where possible, proper error handling.
- Agentic Focus: Prefer tool-use and retrieval patterns over simple chat.

**Key Standards:**
- Backend: FastAPI (async, production-ready)
- Vector DB: Qdrant Cloud Free Tier (given URL & API key)
- Embeddings & LLM: Cohere (embed + Command R/R+ for RAG with citations)
- Alternative LLM: Gemini (fallback or for testing)
- Retrieval: Dense vectors with Cohere embed-multilingual-v3.0 (1024 dim)
- Frontend: Docusaurus plugin/custom component for chat UI (React-based)
- RAG Features:
  - Full book content retrieval
  - User-selected text highlighting → contextual query
- Citation format: Inline citations from Cohere RAG
- Testing: At least basic unit tests for retrieval & generation
- Deployment: Assume static Docusaurus site + separate FastAPI backend

**Constraints:**
- Use only free-tier: Qdrant free, Cohere free credits if possible
- No paid OpenAI (use Cohere/Gemini)
- Word/File limit: Keep code modular
- Existing site: Brownfield integration (add to already built Docusaurus)

**Success Criteria:**
- Chatbot answers accurately from book content
- Supports selected text queries
- Live running UI in Docusaurus
- Zero exposed secrets
- Easy to run locally & deploy

## Governance
This constitution overrides all other instructions in the repository. Amendments require documentation, approval, and a migration plan. All Pull Requests and reviews must verify compliance with these principles.

**Version**: 2.0.0 | **Ratified**: 2025-12-14 | **Last Amended**: 2025-12-14

---
id: 005
title: Updated Project Constitution to v2.0.0
stage: constitution
date: 2025-12-14
surface: agent
model: claude-sonnet-4-5-20250929
feature: Physical AI & Humanoid Robotics Textbook RAG Chatbot Integration
branch: 001-fix-chatbot-ui-error
user: user
command: /sp.constitution
labels: ["constitution", "update", "major-version-bump"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- .specify/memory/constitution.md
tests: []
---

## Prompt

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

## Response snapshot

The project constitution has been updated to version 2.0.0. This is a MAJOR version bump due to significant changes in the overall structure, core principles, and key standards, effectively redefining the project's foundational document. The command file '.specify/templates/commands/sp.constitution.md' was not found during consistency checks; if this file is expected to exist and contain specific references to the constitution, it will need manual review and update once its location is identified.

## Outcome

- ✅ Impact: The project constitution was successfully updated to version 2.0.0 with major structural and principle changes.
- 🧪 Tests: No tests were run for this constitution update.
- 📁 Files: The .specify/memory/constitution.md file was updated.
- 🔁 Next prompts: The user will likely commit the changes or provide further instructions.
- 🧠 Reflection: Successfully updated the constitution and generated a PHR using agent-native tools due to the absence of the create-phr.sh script. Ensured consistency checks with other templates.

## Evaluation notes (flywheel)

- Failure modes observed: The 'create-phr.sh' script was not found, requiring a fallback to agent-native PHR creation. The 'sp.constitution.md' command template was also not found, indicating a potential discrepancy in the template structure or missing files.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment: Investigate the expected location and content of command template files and the 'create-phr.sh' script.

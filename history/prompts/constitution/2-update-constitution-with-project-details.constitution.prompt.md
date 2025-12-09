---
id: 2
title: Update Constitution with Project Details
stage: constitution
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5-20251101
feature: none
branch: main
user: user
command: /sp.constitution
labels: ["constitution", "update"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - Constitution content is updated
---

## Prompt

/sp.constitution/sp.constitution
Project Name: SpecKit-Book-RAG-Chatbot-Docusaurus
Goal: Build and deploy a production-ready RAG-powered chatbot on my existing Docusaurus website for GIAIC Q4 Hackathon submission. The chatbot must intelligently answer questions from my complete AI book (/docs folder ke saare .mdx files) se Roman Urdu + English mix mein perfect jawab de – 5-8 seconds mein + source link ke saath.

Core Principles & Constraints:
1. Source of truth: Only the .mdx files inside /docs folder of my current Docusaurus site
2. Embeddings: Cohere Free Tier (multilingual model – supports English + Urdu romanized)
3. Vector Database: Qdrant running locally in Docker (later optional cloud)
4. Backend Framework: FastAPI (async) + OpenAI Agents SDK (latest version)
5. LLM: Groq (Llama-3.1-70b or Mixtral-8x22b – fastest & free tier) OR OpenAI gpt-4o-mini
6. Frontend: Pure Docusaurus 3 – NO Next.js
7. Chat UI: Embedded React component inside Docusaurus (right sidebar or floating bubble)
8. Code Style: Clean Python OOP, Pydantic v2 models, proper typing, async/await everywhere
9. Full SpecKit Plus workflow must be followed – no shortcuts
10. Zero vibe coding – every step documented

Success Criteria for Hackathon Judges:
- Open my live Docusaurus site
- Ask any complex question from the book (e.g., “Chapter 8 mein ReAct loop kaise implement karte hain?”)
- Chatbot replies correctly within 5–8 seconds WITH source chapter + exact heading/link

Tech Stack (FINAL):
├── Frontend: Docusaurus 3 (already deployed)
├── Chat Component: React + Tailwind inside Docusaurus plugin/custom component
├── Backend: FastAPI + Uvicorn
├── Agents: OpenAI Agents SDK v0.28+
├── Embeddings: Cohere (free trial key)
├── Vector Store: Qdrant (Docker)
├── Loader: LangChain DirectoryLoader + MDX support
└── Deployment: Frontend → Vercel/Netlify (existing), Backend → Railway or Render

## Response snapshot

The project constitution has been updated. Merge conflicts in `.specify/memory/constitution.md` were resolved, and the file was updated with the new project goals, tech stack, folder structure, core principles, success criteria, and governance. The constitution version has been incremented to 1.0.1.

## Outcome

- ✅ Impact: Project constitution updated and merge conflicts resolved.
- 🧪 Tests: Constitution content updated in .specify/memory/constitution.md
- 📁 Files: .specify/memory/constitution.md
- 🔁 Next prompts: Continue with `/sp.implement` or other project phases.
- 🧠 Reflection: Successfully updated the constitution based on user input, ensuring alignment with project goals and technical specifications.

## Evaluation notes (flywheel)

- Failure modes observed: Attempted to use bash script for PHR creation, which failed due to environment issues. Successfully fell back to agent-native PHR creation. Repeated errors in using `Write` tool to populate PHR.
- Graders run and results (PASS/FAIL): None
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A

# Project Specification

**Project Title:** Integrated RAG Chatbot for Physical AI & Humanoid Robotics Textbook
**Requirement:** Hackathon Requirement 2 – Embed RAG Chatbot in published Docusaurus book site

**Functional Requirements:**
1. User poori book ke content pe questions kar sake
2. User selected text highlight karke uspe specific question kar sake (contextual RAG)
3. Answers sirf book ke content se aaye – no hallucination
4. Chat UI book website (Docusaurus) mein smoothly embedded ho
5. Real-time streaming responses (Mandatory for responsive UX)

**Non-Functional Requirements:**
- Backend: FastAPI (Vercel Serverless Functions compatible)
- Vector Database: Qdrant Cloud Free Tier
- Embeddings: Cohere embed-multilingual-v3.0
- Primary LLM: Gemini 2.0 Flash (with built-in RAG + citation support)
pure Node.js + Python hybrid deploy on Vercel
- All secrets environment variables se manage hon
- Fast cold start (<2s on Vercel)
- Free tier limits ka khayal rakha jaye

**Tech Stack & Deployment:**
- Frontend: Existing Docusaurus site (React + MDX)
- Chat UI: Custom React component (using streaming fetch)
- Backend: FastAPI running as Vercel Serverless Function (Python)
- Vector Store: Qdrant Cloud
- Deployment Platform: Vercel (monorepo – frontend + api folder)

**Out of Scope:**
- User authentication
- Chat history persistence
- Multi-language support beyond English/Urdu mix in book

**Acceptance Criteria:**
- Chatbot book ke kisi bhi chapter se accurate answers de
- Selected text pe question ka jawab sirf us text se de
- Live demo Vercel URL pe chal raha ho
- No API keys exposed in frontend
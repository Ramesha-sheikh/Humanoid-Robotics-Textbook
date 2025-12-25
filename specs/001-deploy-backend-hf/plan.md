# Implementation Plan: Deploy Backend to Hugging Face Spaces

**Branch**: `001-deploy-backend-hf` | **Date**: 2025-12-25 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/001-deploy-backend-hf/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Deploy the existing FastAPI backend (providing RAG chatbot and Urdu translation endpoints) to Hugging Face Spaces for production access. The backend currently runs on localhost:8001 and needs to be containerized using Docker, configured with secure environment variables, and made publicly accessible via HTTPS. The Vercel-deployed frontend will be updated to call the production backend URL instead of localhost. CORS will be configured to allow cross-origin requests from the production Vercel domain.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: FastAPI, uvicorn, cohere (v4.37), qdrant-client (v1.7.0), pydantic, python-dotenv, pyyaml
**Storage**: Qdrant Cloud (vector database, already set up with 280 embedded data points)
**Testing**: Manual API testing via curl/Postman, health endpoint verification
**Target Platform**: Hugging Face Spaces (Docker SDK, Linux container on port 7860)
**Project Type**: Web backend API (existing code deployment, no new development)
**Performance Goals**: API responses within 3 seconds average, translation within 10 seconds, 95% success rate
**Constraints**: HF Spaces free tier resources, CORS must allow Vercel domain, no exposed secrets in code/logs
**Scale/Scope**: Low-medium traffic (educational textbook site), 4 API endpoints (/health, /chat, /stream-chat, /translate)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle Compliance

| Principle | Status | Notes |
|-----------|--------|-------|
| **Accuracy** | ✅ PASS | Backend uses existing Cohere RAG with Qdrant embeddings - no changes to accuracy |
| **Privacy & Security** | ✅ PASS | All API keys stored in HF Spaces environment variables, never in code |
| **Reproducibility** | ✅ PASS | Dockerfile ensures consistent deployment, all dependencies versioned |
| **Clarity** | ✅ PASS | Deployment docs will be created (README.md, .env.example) |
| **Rigor** | ✅ PASS | Existing backend already uses async FastAPI with proper error handling |
| **Agentic Focus** | ✅ PASS | No changes to retrieval patterns - maintaining current tool-use approach |

### Key Standards Compliance

| Standard | Status | Notes |
|----------|--------|-------|
| **Backend: FastAPI** | ✅ PASS | Already using FastAPI, no changes needed |
| **Vector DB: Qdrant Cloud** | ✅ PASS | Already configured with 280 data points |
| **Embeddings & LLM: Cohere** | ✅ PASS | Using embed-english-v3.0 and command-r-08-2024 |
| **Testing** | ⚠️ PARTIAL | Manual API testing only (no unit tests for deployment) |
| **Deployment** | ✅ PASS | Separating static Docusaurus + FastAPI backend as intended |
| **Zero exposed secrets** | ✅ PASS | HF Spaces environment variables, .gitignore configured |

### Constraint Compliance

| Constraint | Status | Notes |
|------------|--------|-------|
| **Free-tier only** | ✅ PASS | Using HF Spaces free tier, existing Qdrant/Cohere free accounts |
| **No paid OpenAI** | ✅ PASS | Using Cohere (already configured) |
| **Modular code** | ✅ PASS | Existing backend already modular |
| **Brownfield integration** | ✅ PASS | Deploying existing backend, frontend already built |

### Gate Status: ✅ PASSED

Minor note: Testing is manual (API endpoint verification) rather than automated unit tests. This is acceptable for deployment task as we're not modifying functionality - just deploying existing tested code. Health checks will verify correct deployment.

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
rag-backend/chatbot/              # Backend to be deployed
├── app.py                         # Main FastAPI application (existing)
├── config.py                      # Configuration management (existing)
├── requirements.txt               # Python dependencies (TO CREATE)
├── Dockerfile                     # HF Spaces deployment config (TO CREATE)
├── README.md                      # API documentation (TO CREATE)
└── .env.example                   # Environment variable template (TO CREATE)

my-website/                        # Frontend (already deployed on Vercel)
├── src/
│   ├── theme/DocItem/Content/index.js      # Translation component (UPDATE URL)
│   └── components/LanguageToggle/index.tsx # Language toggle (UPDATE URL)
└── backend/server.js              # Old Gemini backend (NOT USED - can be removed later)

.gitignore                         # Ensure .env files ignored (VERIFY)
```

**Structure Decision**: This is a **web application deployment** where the existing FastAPI backend (`rag-backend/chatbot/`) will be containerized and deployed to Hugging Face Spaces. The frontend (already deployed on Vercel) will be updated to call the production backend URL. No new code modules are being created - only deployment configuration files (Dockerfile, requirements.txt, README.md) and frontend URL updates.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

N/A - All constitution checks passed. No violations to justify.

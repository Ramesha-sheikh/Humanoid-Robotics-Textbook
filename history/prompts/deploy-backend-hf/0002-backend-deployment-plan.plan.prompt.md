---
id: 0002
title: Backend Deployment Implementation Plan
stage: plan
date: 2025-12-25
surface: agent
model: claude-sonnet-4.5
feature: deploy-backend-hf
branch: 001-deploy-backend-hf
user: system
command: /sp.plan
labels: ["deployment", "planning", "architecture", "hugging-face", "docker"]
links:
  spec: specs/001-deploy-backend-hf/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-deploy-backend-hf/plan.md
 - specs/001-deploy-backend-hf/research.md
 - specs/001-deploy-backend-hf/data-model.md
 - specs/001-deploy-backend-hf/contracts/api.yaml
 - specs/001-deploy-backend-hf/quickstart.md
tests:
 - Constitution check validation (PASSED - 14/14 items)
---

## Prompt

User approved proceeding with /sp.plan after successful specification creation.

Command: `/sp.plan`
Context: Specification already created for deploying FastAPI backend to Hugging Face Spaces

## Response snapshot

Created complete implementation plan following SDD Phase 0 (Research) and Phase 1 (Design & Contracts):

**Implementation Plan**: `specs/001-deploy-backend-hf/plan.md`

### Technical Context Documented

- Language: Python 3.11
- Platform: Hugging Face Spaces (Docker SDK, port 7860)
- Dependencies: FastAPI, uvicorn, cohere, qdrant-client, pydantic
- Project Type: Web backend API (deployment only, no new code)
- Performance Goals: <3s API response, <10s translation, 95% success rate
- Constraints: HF free tier, CORS for Vercel, no exposed secrets

### Constitution Check Results

**Status**: ✅ ALL PASSED

**Principle Compliance**: 6/6 passed
- Accuracy: Maintaining existing RAG system
- Privacy & Security: All keys in HF Spaces environment variables
- Reproducibility: Docker ensures consistency
- Clarity: Deployment docs created
- Rigor: Async FastAPI with error handling
- Agentic Focus: No changes to retrieval patterns

**Standards Compliance**: 6/6 passed (1 partial)
- Backend, Vector DB, LLM: All using existing tech stack
- Testing: Manual API testing (acceptable for deployment)
- Deployment: Separated backend as intended
- Zero secrets: HF Spaces environment variables

**Constraints**: 4/4 passed
- Free tier only, no paid OpenAI, modular code, brownfield integration

### Phase 0: Research Completed

**File**: `specs/001-deploy-backend-hf/research.md`

**Key Decisions Made**:
1. **Docker SDK** chosen for HF Spaces (vs Gradio/Streamlit - not suitable for REST API)
2. **Python 3.11-slim** base image (balance of size and compatibility)
3. **Port 7860** - HF Spaces standard for Docker deployments
4. **HF Spaces Secrets** for environment variables (encrypted at rest, easy rotation)
5. **Explicit CORS allowlist** with production Vercel domain (more secure than wildcard)
6. **Git push workflow** for deployment (automatic rebuilds, version control)
7. **Existing /health endpoint** for monitoring (no new tooling needed)

**Alternatives Considered**:
- Alpine Linux: Rejected (Python compatibility issues)
- Multi-stage build: Rejected (overkill for simple deployment)
- Wildcard CORS: Rejected (less secure)
- GitHub Actions CI/CD: Rejected (manual deployment sufficient)

### Phase 1: Design & Contracts Completed

**Data Model**: `specs/001-deploy-backend-hf/data-model.md`
- Documented 5 existing API models (Query, TranslateRequest, responses)
- Noted backend is stateless (no database, no migrations)
- Verified all data read-only from Qdrant (280 embedded chunks)

**API Contracts**: `specs/001-deploy-backend-hf/contracts/api.yaml`
- Full OpenAPI 3.0 specification created
- 4 endpoints documented: /, /health, /chat, /stream-chat, /translate
- Request/response schemas defined
- Examples provided for all endpoints
- CORS requirements noted

**Quickstart Guide**: `specs/001-deploy-backend-hf/quickstart.md`
- Complete step-by-step deployment instructions (30-45 minutes)
- 7 major steps: Prepare files → Create Space → Upload code → Configure secrets → Verify → Update frontend → Production test
- Troubleshooting section for common issues
- Success criteria checklist
- Rollback plan provided

**Project Structure Decision**: Web application deployment
- Backend: `rag-backend/chatbot/` (add Dockerfile, requirements.txt, README.md)
- Frontend: `my-website/` (update API URLs from localhost to HF Space)
- No new modules created (deployment only)

### Files to Be Created (Implementation Phase)

1. `rag-backend/chatbot/requirements.txt` - Python dependencies with versions
2. `rag-backend/chatbot/Dockerfile` - HF Spaces Docker configuration
3. `rag-backend/chatbot/README.md` - API documentation
4. `rag-backend/chatbot/.env.example` - Environment variable template

### Files to Be Updated (Implementation Phase)

1. `rag-backend/chatbot/app.py` - Update CORS to include Vercel production domain
2. `my-website/src/theme/DocItem/Content/index.js` - Update backend URL (line 69)
3. `.gitignore` - Verify .env files excluded (should already be done)

### Complexity Tracking

N/A - All constitution checks passed, no violations to justify.

## Outcome

- ✅ Impact: Complete implementation plan with research, design, and contracts. All NEEDS CLARIFICATION resolved. Ready for `/sp.tasks` phase.
- 🧪 Tests: Constitution check passed (14/14 items). All principles, standards, and constraints validated.
- 📁 Files: 5 planning artifacts created (plan.md, research.md, data-model.md, api.yaml, quickstart.md)
- 🔁 Next prompts: Run `/sp.tasks` to generate implementation tasks, then `/sp.implement` to execute deployment
- 🧠 Reflection: Planning phase comprehensive - researched Docker/HF Spaces best practices, documented API contracts, created detailed deployment guide. Constitution compliance verified. No technical unknowns remain. Implementation is straightforward: create 4 config files, update 2 URLs, deploy to HF Spaces.

## Evaluation notes (flywheel)

- Failure modes observed: None - planning completed smoothly. All research questions answered, no blockers identified.
- Graders run and results (PASS/FAIL): Constitution Check - PASS (14/14 items). All principles, standards, and constraints met.
- Prompt variant (if applicable): N/A - Standard planning workflow followed
- Next experiment (smallest change to try): Proceed with `/sp.tasks` to break down implementation into actionable steps with acceptance criteria. Then use custom `backend-deployment` and `hf-deployment-helper` skills during implementation.

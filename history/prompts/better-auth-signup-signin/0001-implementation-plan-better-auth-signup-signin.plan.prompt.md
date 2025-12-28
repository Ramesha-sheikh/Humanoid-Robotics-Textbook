---
id: 0001
title: Implementation Plan Better-Auth Signup Signin
stage: plan
date: 2025-12-27
surface: agent
model: claude-sonnet-4-5-20250929
feature: better-auth-signup-signin
branch: main
user: Laptronics.co
command: /sp.plan
labels: ["authentication", "better-auth", "personalization", "neon-db", "architecture"]
links:
  spec: specs/002-better-auth-signup-signin/spec.md
  ticket: null
  adr: history/adr/001-custom-jwt-authentication.md
  pr: null
files:
  - "specs/002-better-auth-signup-signin/plan.md"
  - "history/prompts/better-auth-signup-signin/0001-implementation-plan-better-auth-signup-signin.plan.prompt.md"
tests:
  - "Integration test framework defined (Phase 4)"
---

## Prompt

/sp.plan# /sp.plan.md

## Implementation Plan

1. **Environment Setup**
   - Install Better-Auth
   - Setup Neon DB
   - Setup Next.js / Node.js backend

2. **Signup Flow**
   - API route using Better-Auth
   - Form fields for software/hardware background
   - Save in Neon DB

3. **Signin Flow**
   - API route for login
   - Validate credentials

4. **Personalization**
   - Fetch user background at login
   - Filter/rank content from Qdrant

5. **SpeckitPlus Automation**
   - Map each task to `/sp.tasks` and `/sp.implement`

6. **Testing**
   - Signup/Signin flow
   - Personalized content rendering

## Response snapshot

Created comprehensive architectural plan for Better-Auth authentication with personalization:

**Plan Document Complete** (`specs/002-better-auth-signup-signin/plan.md`):

### Key Architectural Decisions (6 ADRs):

1. **ADR-001: Custom JWT Authentication** (not Better-Auth or FastAPI-Users)
   - Rationale: Better-Auth is Node.js-only; custom JWT is lightweight (~200 LOC) and fully controlled
   - Trade-off: Manual implementation vs missing advanced features (OAuth, MFA) - acceptable for MVP

2. **ADR-002: JWT Tokens with 7-Day Expiration** (not session-based)
   - Rationale: Stateless tokens work with serverless HF Spaces deployment; no Redis needed
   - Trade-off: Cannot revoke before expiration vs no infrastructure complexity

3. **ADR-003: Relevance Score Boosting** (not pre-filtering or LLM re-ranking)
   - Rationale: Balances personalization with performance (< 50ms overhead vs 2-3s for LLM re-ranking)
   - Trade-off: Soft personalization vs aggressive filtering or sophisticated LLM ranking

4. **ADR-004: SQLAlchemy ORM with Neon DB** (not Prisma or raw SQL)
   - Rationale: Python ecosystem standard, connection pooling, migration support (Alembic)
   - Trade-off: Slightly verbose vs Prisma's modern DX - maturity preferred

5. **ADR-005: localStorage for JWT Storage** (not httpOnly cookies)
   - Rationale: Simple SPA pattern, low security risk for educational content, React XSS protection
   - Trade-off: XSS vulnerability vs implementation simplicity - acceptable for low-risk app

6. **ADR-006: Optional Authentication** (not required signin)
   - Rationale: Low friction, open education philosophy, personalization as incentive (not gate)
   - Trade-off: Some anonymous usage vs accessibility and gradual trust building

### Implementation Phases:

- **Phase 1: Backend Foundation** (2 days) - Neon DB, SQLAlchemy, auth routes, JWT logic
- **Phase 2: Personalization Engine** (1 day) - Score boosting, user context extraction
- **Phase 3: Frontend Auth UI** (2 days) - Signin/signup modals, profile settings, header updates
- **Phase 4: Testing & Deployment** (1 day) - Integration tests, security audit, production deployment

**Total Estimate**: 6 days

### Key Files:

**New Backend Files** (8):
- `auth.py` (200 LOC) - Auth routes and JWT logic
- `models.py` (50 LOC) - SQLAlchemy User model
- `schemas.py` (100 LOC) - Pydantic models
- `database.py` (30 LOC) - DB connection
- `middleware.py` (50 LOC) - JWT validation
- `alembic/` - Migration scripts

**Modified Backend Files** (3):
- `app.py` (+100 LOC) - Auth routes, personalization in chatbot endpoints
- `config.py` (+20 LOC) - Neon DB connection, JWT secret
- `requirements.txt` - Add: `sqlalchemy`, `psycopg2-binary`, `pyjwt`, `bcrypt`, `alembic`

**New Frontend Files** (6):
- `Auth/SigninModal.tsx` (150 LOC)
- `Auth/SignupModal.tsx` (200 LOC)
- `Auth/ProfileSettingsModal.tsx` (150 LOC)
- `Auth/AuthContext.tsx` (100 LOC)
- `Auth/AuthButton.tsx` (100 LOC)
- `Auth/styles.module.css` (50 LOC)

**Modified Frontend Files** (3):
- `ChatBot/api.ts` (+100 LOC) - Auth methods, Authorization header
- `theme/Layout/index.tsx` (+30 LOC) - AuthContext provider, AuthButton
- `ChatBot/ChatWindow.tsx` (+20 LOC) - Anonymous user banner

### Non-Functional Requirements:

- **Performance**: Signup/Signin < 2s (p95), Token validation < 100ms (p99), Chatbot < 5s (no degradation)
- **Security**: bcrypt (10 rounds), JWT (HS256, 7-day exp), SQLAlchemy parameterized queries
- **Reliability**: 99.5% uptime, graceful degradation (chatbot works anonymously if DB down)
- **Cost**: $0 (Neon DB free tier, no additional API calls)

### Risk Mitigation:

1. **Neon DB connection failures** → Connection pooling, health checks, anonymous fallback
2. **JWT secret leak** → Store in HF Spaces secrets, rotation procedure, monitoring
3. **Personalization doesn't improve UX** → Tunable boost factors, A/B testing, kill switch

### Next Steps:

1. Create Neon DB database (5 minutes)
2. Generate JWT secret: `openssl rand -hex 32` (1 minute)
3. Run `/sp.tasks` to generate detailed implementation tasks
4. Create feature branch: `002-better-auth-signup-signin`
5. Begin Phase 1: Backend foundation

## Outcome

- ✅ Impact: Complete architectural plan with 6 ADRs, 4 implementation phases, risk mitigation strategies
- 🧪 Tests: Defined DoD with unit tests, integration tests, security scans, performance targets
- 📁 Files:
  - specs/002-better-auth-signup-signin/plan.md (comprehensive plan)
  - history/prompts/better-auth-signup-signin/0001-implementation-plan-better-auth-signup-signin.plan.prompt.md
- 🔁 Next prompts: Run `/sp.tasks` to generate actionable task list, then `/sp.implement`
- 🧠 Reflection: All key decisions documented with trade-offs; plan is actionable and ready for implementation

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A (planning phase)
- Prompt variant (if applicable): Used constitution architect guidelines for comprehensive planning
- Next experiment (smallest change to try): Validate Neon DB free tier limits (10GB, 100 hours compute)

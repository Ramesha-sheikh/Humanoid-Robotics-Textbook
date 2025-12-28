# Implementation Plan: Better-Auth Signup & Signin with User Personalization

**Feature Branch**: `002-better-auth-signup-signin`
**Created**: 2025-12-27
**Status**: Draft
**Reference**: [spec.md](./spec.md), [data-model.md](./data-model.md), [contracts/api.md](./contracts/api.md)

---

## 1. Scope and Dependencies

### In Scope

#### Authentication System
- FastAPI authentication routes (`/auth/signup`, `/auth/signin`, `/auth/me`, `/auth/signout`)
- JWT token generation and validation using PyJWT
- Password hashing with bcrypt (10 rounds minimum)
- Session management with 7-day token expiration
- Email/password validation with security requirements

#### User Profile Management
- Neon DB PostgreSQL database setup with `users` table
- SQLAlchemy ORM integration for database operations
- User profile schema: `id`, `email`, `password_hash`, `programming_experience`, timestamps
- Profile update endpoint (`PUT /auth/me`)

#### Personalization Engine
- Extract user context from JWT tokens in chatbot endpoints
- Implement relevance score boosting algorithm for Qdrant results
- Language-specific boosting: Python (1.3x), C++ (1.3x), None (1.2x conceptual), Both (balanced)
- Modify `/chat` and `/stream-chat` to accept optional `Authorization` header
- Return personalization metadata in responses (`personalized: true/false`)

#### Frontend Integration
- React auth UI components (SigninModal, SignupModal, ProfileSettingsModal)
- Header with conditional rendering (anonymous vs authenticated state)
- JWT token storage in localStorage
- API client modifications to include `Authorization` header
- Anonymous user banner promoting signup

### Out of Scope

- Social authentication (Google, GitHub OAuth) - future phase
- Email verification for new signups - trust-based MVP
- Password reset functionality - deferred to phase 2
- Multi-factor authentication (MFA) - security enhancement for later
- Advanced personalization (ML-based, collaborative filtering) - start simple
- Additional profile fields (ROS2 experience, hardware access) - phase 2
- Rate limiting and abuse prevention - post-MVP hardening
- Token blacklisting for signout - stateless JWT approach
- Mobile app authentication - web-only for now

### External Dependencies

| Dependency | Version | Purpose | Owner | Risk Level |
|------------|---------|---------|-------|------------|
| **Neon DB** | PostgreSQL 15 | User account storage | External (Neon) | Low - serverless, managed |
| **SQLAlchemy** | 2.0+ | Python ORM for database | Internal (FastAPI) | Low - mature library |
| **PyJWT** | 2.8+ | JWT token generation | Internal (FastAPI) | Low - standard library |
| **bcrypt** | 4.0+ | Password hashing | Internal (FastAPI) | Low - industry standard |
| **python-multipart** | 0.0.6+ | FastAPI form parsing | Internal (FastAPI) | Low - FastAPI dependency |
| **Qdrant** | 1.7.0 (existing) | Vector database (unchanged) | External (Qdrant Cloud) | None - no changes |
| **Cohere** | 4.37 (existing) | Embeddings & LLM (unchanged) | External (Cohere) | None - no changes |
| **React** | 19.0.0 (existing) | Frontend UI framework | Internal (Docusaurus) | Low - existing setup |

### Internal Dependencies

| Component | Location | Impact | Modification Type |
|-----------|----------|--------|-------------------|
| **FastAPI Backend** | `rag-backend/chatbot/app.py` | High | Extend with auth routes |
| **Config Module** | `rag-backend/chatbot/config.py` | Medium | Add DB connection config |
| **Qdrant Query Logic** | `rag-backend/chatbot/app.py` (search functions) | Medium | Add score boosting logic |
| **Frontend API Client** | `my-website/src/components/ChatBot/api.ts` | High | Add auth methods + headers |
| **Frontend Header** | `my-website/src/theme/Layout/index.tsx` | Medium | Add auth UI components |
| **CORS Configuration** | `rag-backend/chatbot/app.py` (middleware) | Low | Already configured correctly |

---

## 2. Key Decisions and Rationale

### Decision 1: Use Custom JWT Auth with FastAPI (Not Better-Auth)

**Options Considered**:
1. **Better-Auth** (as originally specified)
   - Pros: Modern, well-documented, comprehensive features
   - Cons: Designed for Node.js/Next.js; no official Python/FastAPI support
2. **FastAPI-Users** (community library)
   - Pros: FastAPI-specific, handles auth + user management + OAuth
   - Cons: Heavy abstraction, opinionated structure, potential overkill for simple use case
3. **Custom JWT Implementation** (PyJWT + bcrypt) ✅ **SELECTED**
   - Pros: Lightweight, full control, easy to understand and debug, no vendor lock-in
   - Cons: Manual implementation of auth logic (token generation, validation, middleware)

**Rationale**:
- Better-Auth lacks Python support (ecosystem mismatch)
- FastAPI-Users introduces unnecessary complexity for our simple auth requirements (email/password + one profile field)
- Custom JWT implementation aligns with "smallest viable change" principle from constitution
- PyJWT and bcrypt are battle-tested, industry-standard libraries
- Full control allows optimization for personalization use case (embed user context in JWT payload)
- Implementation complexity is low (~200 lines of auth code)

**Trade-offs**:
- ✅ Simplicity and control vs ❌ Missing advanced features (OAuth, MFA) - acceptable for MVP
- ✅ No vendor lock-in vs ❌ Manual security maintenance - mitigated by using standard libraries
- ✅ Optimized for our use case vs ❌ Re-inventing some wheels - acceptable trade-off

**Principles Applied**:
- Smallest viable change (custom implementation is ~200 LOC vs 1000+ with FastAPI-Users)
- Prefer explicit over implicit (manual JWT logic is transparent)
- Separation of concerns (auth is isolated module in backend)

**ADR**: [ADR-001: Custom JWT Authentication over Better-Auth or FastAPI-Users](#adr-001-custom-jwt-authentication)

---

### Decision 2: JWT Tokens with 7-Day Expiration (Not Session-Based Auth)

**Options Considered**:
1. **Session-Based Auth** (server-side session storage)
   - Pros: Server controls revocation, more secure for sensitive apps
   - Cons: Requires session store (Redis), state management, scaling complexity
2. **JWT Tokens (Stateless)** ✅ **SELECTED**
   - Pros: No server-side state, scales horizontally, works with serverless backends
   - Cons: Cannot revoke before expiration, token size larger than session ID
3. **Hybrid** (JWT + Token Blacklist)
   - Pros: Best of both worlds (stateless + revocation capability)
   - Cons: Adds complexity (requires Redis/DB for blacklist), overkill for educational textbook

**Rationale**:
- Current backend is deployed on Hugging Face Spaces (serverless, stateless environment)
- No Redis or session store infrastructure available or needed for MVP
- Educational textbook use case has low security risk (no financial data, no PII beyond email)
- 7-day expiration balances convenience (users stay logged in) and security (forced re-auth after inactivity)
- Stateless tokens allow backend to scale without session synchronization complexity

**Trade-offs**:
- ✅ Scalability and simplicity vs ❌ No immediate revocation - acceptable for low-risk app
- ✅ Works with serverless deployment vs ❌ Slightly larger token size - negligible network overhead
- ✅ No additional infrastructure vs ❌ Cannot blacklist compromised tokens - mitigated by short expiration

**Principles Applied**:
- Prefer simplicity over premature optimization (no session store for MVP)
- Align with deployment architecture (stateless Hugging Face Spaces)
- Security proportional to risk (educational content, not banking app)

**ADR**: [ADR-002: JWT Tokens with 7-Day Expiration](#adr-002-jwt-tokens-with-7-day-expiration)

---

### Decision 3: Relevance Score Boosting (Not Pre-Filtering or LLM Re-Ranking)

**Options Considered**:
1. **Pre-Filtering** (filter Qdrant by metadata before search)
   - Pros: Strict language filtering, guarantees relevant results
   - Cons: May exclude valuable content, reduces result diversity, metadata may be incomplete
2. **Post-Search LLM Re-Ranking** (retrieve generic results, re-rank with user context via LLM)
   - Pros: Most sophisticated, accounts for nuanced preferences
   - Cons: Adds 2-3s latency, increases API cost (extra Cohere call), over-engineered for MVP
3. **Relevance Score Boosting** (adjust vector similarity scores by language match) ✅ **SELECTED**
   - Pros: Balanced results, preserves diversity, minimal latency, simple implementation
   - Cons: Less strict than pre-filtering, requires language metadata in payloads

**Rationale**:
- Pre-filtering is too aggressive (user with "Python" preference may still benefit from conceptual C++ explanations)
- LLM re-ranking adds unacceptable latency (chatbot responses must complete in 5s per NFR-007)
- Score boosting provides "soft" personalization - Python users see Python first, but C++ isn't hidden
- Implementation is simple: multiply similarity scores by boost factor (1.3x for language match)
- Falls back gracefully if metadata is missing (treat as language-agnostic content)

**Trade-offs**:
- ✅ Speed and simplicity vs ❌ Less aggressive personalization - acceptable for MVP
- ✅ Preserves content diversity vs ❌ Requires metadata in Qdrant - likely already present
- ✅ Low latency overhead (<50ms) vs ❌ Not as sophisticated as LLM re-ranking - premature optimization

**Principles Applied**:
- Smallest viable change (score boosting is ~30 LOC modification to existing query)
- Performance first (maintain 5s response time per NFR-007)
- Graceful degradation (works even with incomplete metadata)

**ADR**: [ADR-003: Relevance Score Boosting for Personalization](#adr-003-relevance-score-boosting)

---

### Decision 4: SQLAlchemy ORM with Neon DB (Not Prisma or Raw SQL)

**Options Considered**:
1. **Prisma** (TypeScript-first ORM)
   - Pros: Type-safe, excellent DX, auto-migrations
   - Cons: Designed for Node.js; Python support is experimental and limited
2. **Raw SQL with psycopg2** (PostgreSQL adapter)
   - Pros: Maximum control, no abstraction overhead
   - Cons: Manual query construction, SQL injection risk if not careful, verbose code
3. **SQLAlchemy ORM** ✅ **SELECTED**
   - Pros: Python standard, mature, FastAPI-native, type hints support, connection pooling
   - Cons: Learning curve for complex queries, some abstraction overhead

**Rationale**:
- SQLAlchemy is the de facto standard ORM for Python web applications
- FastAPI documentation uses SQLAlchemy in all official examples
- Provides connection pooling out-of-box (required by NFR-008: minimum 5 connections)
- Type hints with Pydantic models ensure type safety comparable to Prisma
- Alembic (SQLAlchemy companion) handles migrations for future schema changes
- Team familiarity (if applicable) and ecosystem maturity reduce risk

**Trade-offs**:
- ✅ Python ecosystem fit vs ❌ Slightly more verbose than Prisma - acceptable
- ✅ Battle-tested and mature vs ❌ Not as modern as Prisma - stability preferred
- ✅ Connection pooling built-in vs ❌ Abstraction overhead - negligible for our scale

**Principles Applied**:
- Use ecosystem-standard tools (SQLAlchemy is Python web standard)
- Prefer mature over trendy (SQLAlchemy has 15+ years of production use)
- Separation of concerns (ORM handles DB, Pydantic handles validation)

**ADR**: [ADR-004: SQLAlchemy ORM with Neon DB](#adr-004-sqlalchemy-orm-with-neon-db)

---

### Decision 5: localStorage for JWT Storage (Not httpOnly Cookies)

**Options Considered**:
1. **httpOnly Cookies** (server-set, inaccessible to JavaScript)
   - Pros: Protected from XSS attacks, more secure
   - Cons: Requires CORS credentials configuration, CSRF protection needed, harder to debug
2. **localStorage** ✅ **SELECTED**
   - Pros: Simple to implement, works with any CORS setup, easy to debug, widely used pattern
   - Cons: Vulnerable to XSS attacks if frontend has injection vulnerabilities
3. **sessionStorage** (cleared when tab closes)
   - Pros: More secure than localStorage (auto-clears), same API
   - Cons: Poor UX (users logged out on tab close), not suitable for 7-day sessions

**Rationale**:
- Educational textbook has low security risk (no financial data, credit cards, or sensitive PII)
- XSS risk is already mitigated by React's default escaping and sanitization
- Docusaurus framework provides built-in XSS protection
- localStorage is standard pattern for SPAs with JWT auth (Auth0, Firebase use this approach)
- Simplifies CORS configuration (no need for `credentials: true`)
- Easier to implement and debug (inspect localStorage in DevTools)

**Trade-offs**:
- ✅ Simplicity and developer experience vs ❌ XSS vulnerability - acceptable for low-risk app
- ✅ Works with existing CORS setup vs ❌ Less secure than httpOnly cookies - risk is proportional to sensitivity
- ✅ Easier to debug vs ❌ Requires frontend XSS protection - already have via React

**Principles Applied**:
- Security proportional to risk (educational content, not banking app)
- Prefer simplicity over premature optimization (httpOnly cookies are overkill)
- Trust existing framework protections (React + Docusaurus handle XSS)

**ADR**: [ADR-005: localStorage for JWT Token Storage](#adr-005-localstorage-for-jwt-storage)

---

### Decision 6: Optional Authentication (Not Required for Chatbot Access)

**Options Considered**:
1. **Required Authentication** (force signin before chatbot access)
   - Pros: Complete user tracking, higher signup rate once committed
   - Cons: High friction, reduces engagement, may deter casual users
2. **Optional Authentication with Benefits** ✅ **SELECTED**
   - Pros: Low friction, users see value before committing, gradual trust building
   - Cons: Some anonymous usage (no user data), potential for abuse
3. **Freemium Model** (limited anonymous access, unlimited for signed-in users)
   - Pros: Monetization path, encourages signup
   - Cons: Rate limiting complexity, may frustrate users, overkill for educational content

**Rationale**:
- Constitution principle: "User experience drives content ranking" - forced auth degrades experience
- Educational content should be accessible to all (open education philosophy)
- Users can evaluate chatbot quality before committing to signup
- Personalization serves as carrot (incentive to sign up) rather than stick (forced gate)
- Anonymous usage provides valuable feedback and testing
- Aligns with Docusaurus philosophy (static site, freely accessible documentation)

**Trade-offs**:
- ✅ Accessibility and low friction vs ❌ Some anonymous usage - acceptable for educational mission
- ✅ Users experience value first vs ❌ Lower signup rate initially - long-term trust building
- ✅ No rate limiting needed vs ❌ Potential for abuse - monitor and address if needed

**Principles Applied**:
- User experience drives content ranking (constitution principle #2)
- Accessibility first (educational content should be open)
- Incentivize with value, not forced gates

**ADR**: [ADR-006: Optional Authentication for Chatbot Access](#adr-006-optional-authentication)

---

## 3. Interfaces and API Contracts

### Public APIs

All APIs return JSON responses with consistent error format: `{"detail": "error message"}`.

#### 3.1 Authentication Endpoints

**POST /auth/signup**
- **Input**: `SignupRequest { email: EmailStr, password: str, programming_experience: Literal }`
- **Output**: `AuthResponse { access_token: str, token_type: "bearer", user: UserProfile }`
- **Errors**: `409 Conflict` (duplicate email), `422 Unprocessable Entity` (validation)
- **Validation**:
  - Email: RFC 5322 format, max 255 chars
  - Password: min 8 chars, max 128 chars, must contain uppercase, lowercase, digit
  - Programming experience: one of ["None", "Python", "C++", "Both Python and C++"]
- **Idempotency**: Not idempotent (duplicate email returns 409)
- **Timeout**: 2s (database insert + bcrypt hashing)
- **Reference**: [contracts/api.md#1-post-authsignup](./contracts/api.md#1-post-authsignup)

**POST /auth/signin**
- **Input**: `SigninRequest { email: EmailStr, password: str }`
- **Output**: `AuthResponse { access_token: str, token_type: "bearer", user: UserProfile }`
- **Errors**: `401 Unauthorized` (invalid credentials), `422 Unprocessable Entity` (validation)
- **Validation**: Email format, password non-empty
- **Idempotency**: Idempotent (same result for repeated calls with valid credentials)
- **Timeout**: 2s (database query + bcrypt verification)
- **Security**: Constant-time password comparison to prevent timing attacks
- **Reference**: [contracts/api.md#2-post-authsignin](./contracts/api.md#2-post-authsignin)

**GET /auth/me**
- **Input**: `Authorization: Bearer <token>` header
- **Output**: `UserProfile { id, email, programming_experience, created_at, updated_at }`
- **Errors**: `401 Unauthorized` (missing/invalid/expired token)
- **Validation**: JWT signature verification, expiration check
- **Idempotency**: Idempotent (read-only)
- **Timeout**: 500ms (JWT validation + database query)
- **Reference**: [contracts/api.md#3-get-authme](./contracts/api.md#3-get-authme)

**PUT /auth/me**
- **Input**: `UpdateProfileRequest { programming_experience: Literal }` + `Authorization` header
- **Output**: `UserProfile` (updated)
- **Errors**: `401 Unauthorized`, `422 Unprocessable Entity`
- **Validation**: JWT verification, programming_experience enum value
- **Idempotency**: Idempotent (updating to same value has no effect)
- **Timeout**: 1s (database update + trigger for updated_at)
- **Reference**: [contracts/api.md#4-put-authme](./contracts/api.md#4-put-authme)

**POST /auth/signout**
- **Input**: `Authorization: Bearer <token>` header
- **Output**: `{ message: "Successfully signed out" }`
- **Errors**: `401 Unauthorized`
- **Validation**: JWT verification (optional - client-side signout is sufficient)
- **Idempotency**: Idempotent
- **Timeout**: 200ms (no database operation; JWT validation only)
- **Note**: Stateless JWT means server cannot invalidate token; client deletes from localStorage
- **Reference**: [contracts/api.md#5-post-authsignout](./contracts/api.md#5-post-authsignout)

#### 3.2 Enhanced Chatbot Endpoints

**POST /stream-chat (Enhanced)**
- **Input**: `Query { question: str, selected_text: str, agent_id: str | None }` + **Optional** `Authorization` header
- **Output**: SSE stream with `{ token: str, done: bool, response?: ChatResponse }`
- **ChatResponse** (enhanced): `{ answer: str, sources: list[str], personalized: bool, user_id?: str }`
- **Errors**: `500 Internal Server Error` (Qdrant/Cohere failure)
- **Validation**: Question max 500 chars (reasonable limit)
- **Timeout**: 30s (streaming response, may take longer for complex queries)
- **Personalization Logic**:
  - If `Authorization` header present and valid → extract `programming_experience` from JWT
  - Apply score boosting to Qdrant results based on experience
  - Set `personalized: true` in response metadata
  - If no header or invalid token → anonymous mode, `personalized: false`
- **Reference**: [contracts/api.md#6-post-stream-chat-enhanced](./contracts/api.md#6-post-stream-chat-enhanced-with-auth)

**POST /chat (Enhanced)**
- **Input**: Same as `/stream-chat`
- **Output**: `ChatResponse` (non-streaming, complete response)
- **Errors**: Same as `/stream-chat`
- **Validation**: Same as `/stream-chat`
- **Timeout**: 10s (complete response generation)
- **Personalization Logic**: Identical to `/stream-chat`
- **Reference**: [contracts/api.md#7-post-chat-enhanced](./contracts/api.md#7-post-chat-enhanced)

### Versioning Strategy

- **Current Version**: v1 (implicit, no version in URL path)
- **Future Versions**: If breaking changes needed, introduce `/v2/auth/*` paths
- **Backward Compatibility**: Maintain v1 endpoints indefinitely (no breaking changes to existing contracts)
- **Deprecation Policy**: 6-month notice before removing deprecated endpoints

### Error Taxonomy

| HTTP Status | Error Type | Use Case | Retry Strategy |
|-------------|------------|----------|----------------|
| 400 | Bad Request | Malformed JSON, missing required field | Do not retry (client fix needed) |
| 401 | Unauthorized | Invalid/missing/expired token | Prompt user to sign in again |
| 403 | Forbidden | Valid auth but insufficient permissions | Do not retry (future use) |
| 409 | Conflict | Duplicate email during signup | Do not retry (prompt user to sign in) |
| 422 | Unprocessable Entity | Validation error (invalid email, weak password) | Do not retry (prompt user to fix input) |
| 500 | Internal Server Error | Database connection failure, unexpected exception | Retry with exponential backoff (max 3 attempts) |

---

## 4. Non-Functional Requirements (NFRs) and Budgets

### Performance

| Metric | Target | Measurement Method | Budget |
|--------|--------|-------------------|--------|
| **Signup Latency** (p95) | < 2s | Time from request to response | 500ms bcrypt + 1s DB insert + 500ms JWT generation |
| **Signin Latency** (p95) | < 2s | Time from request to response | 500ms bcrypt + 1s DB query + 500ms JWT generation |
| **Token Validation** (p99) | < 100ms | Time to validate JWT in middleware | 50ms signature verification + 50ms DB query (cache user) |
| **Personalized Chat** (p95) | < 5s | Time to first token in stream | No degradation from current 5s target (NFR-007) |
| **Score Boosting Overhead** | < 50ms | Difference between generic and personalized search | O(n) multiplication on top-k results (k=20) |
| **Database Connection Pool** | Min 5 connections | SQLAlchemy pool size | Handle 5 concurrent authenticated requests |
| **Throughput** | 50 requests/second | Sustained load testing | Sufficient for educational use case (low traffic) |

**Performance Budgets**:
- Auth endpoints: 2s total budget (1s DB, 500ms crypto, 500ms network)
- Chatbot personalization: No additional latency beyond current 5s
- Profile updates: 1s budget (fast feedback for user)

### Reliability

| Requirement | Target | Measurement | Mitigation |
|-------------|--------|-------------|------------|
| **Uptime** | 99.5% | Weekly monitoring | Rely on Neon DB (99.9% SLA) + HF Spaces redundancy |
| **Auth Success Rate** | 99.9% | Valid credentials always succeed | Database replication, connection pooling |
| **Graceful Degradation** | 100% chatbot availability | If Neon DB down, anonymous mode still works | Catch DB errors, return personalized=false |
| **Token Expiration Handling** | No 500 errors | Expired tokens return 401, not crash | Explicit exp check in JWT validation |
| **Database Connection Failures** | Retry 3 times | Exponential backoff (100ms, 500ms, 1s) | SQLAlchemy auto-retry with `pool_pre_ping=True` |

**SLOs** (Service Level Objectives):
- Auth endpoints: 99.5% success rate for valid credentials
- Chatbot endpoints: 99.9% success rate (including anonymous fallback)
- Database queries: p95 < 200ms (Neon DB should meet this)

**Error Budget**:
- Allowed downtime: 3.6 hours/month (99.5% uptime)
- Allowed auth failures: 0.1% (1 in 1000 valid attempts may fail due to transient errors)

### Security

| Requirement | Implementation | Verification |
|-------------|----------------|--------------|
| **Password Hashing** | bcrypt with 10 rounds (2^10 iterations) | Unit test verifies hash format and rounds |
| **JWT Secret** | 256-bit random secret from environment variable | Check `JWT_SECRET_KEY` length >= 32 chars |
| **Token Expiration** | 7 days (604800 seconds) | JWT `exp` claim verified on every request |
| **SQL Injection Prevention** | SQLAlchemy parameterized queries | No raw SQL; use ORM methods only |
| **XSS Prevention** | React auto-escaping + CSP headers | Docusaurus default CSP config |
| **CORS Restrictions** | Whitelist only production + dev origins | Verify `allow_origins` list |
| **Rate Limiting** (future) | Not implemented in MVP | Monitor abuse, add if needed |
| **Password Strength** | Min 8 chars, uppercase, lowercase, digit | Pydantic validator + frontend hints |
| **Constant-Time Comparison** | Use `secrets.compare_digest()` for password check | Prevent timing attacks |
| **HTTPS Only** | All production traffic via HTTPS | HF Spaces enforces HTTPS |

**Security Audits**:
- Manual code review: Check for SQL injection, XSS, CSRF (no state = no CSRF risk)
- Dependency audit: `pip-audit` to scan for known vulnerabilities
- Penetration testing: Not required for MVP (low-risk educational app)

**Data Handling**:
- **PII**: Email address only (minimal PII, no phone numbers, addresses, etc.)
- **Encryption at Rest**: Neon DB handles encryption (AES-256)
- **Encryption in Transit**: HTTPS for all API calls
- **Data Retention**: Indefinite (users can delete account in future phase)
- **GDPR Compliance** (future): Add account deletion endpoint, data export

### Cost

**Estimated Monthly Costs** (for 1000 active users):

| Service | Usage | Cost | Notes |
|---------|-------|------|-------|
| **Neon DB** | 10GB storage, 100 hours compute | $0 (free tier) | Free tier: 10GB storage, 100 hours/month |
| **Hugging Face Spaces** | Existing backend (no change) | $0 (current setup) | Already deployed; auth adds minimal CPU |
| **Cohere API** | No change (same query volume) | $0 (existing budget) | Personalization doesn't add API calls |
| **Qdrant** | No change (same collection) | $0 (existing budget) | Score boosting is client-side logic |
| **JWT Operations** | CPU-only (no external service) | $0 | Negligible CPU overhead |

**Unit Economics**:
- **Cost per User**: $0 (all services within free tiers)
- **Cost per Auth Request**: $0 (no external auth service)
- **Cost per Personalized Chat**: $0 (no additional API calls)

**Scaling Thresholds**:
- Neon DB free tier limits: 10GB storage (est. ~100k users), 100 compute hours/month
- If exceeded: Upgrade to Neon Pro ($19/month for 100GB storage)
- No other cost increases expected (Cohere and Qdrant usage unchanged)

---

## 5. Data Management and Migration

### Source of Truth

- **User Accounts**: Neon DB PostgreSQL (single source of truth)
- **User Sessions**: JWT tokens (client-side storage, server validates signature)
- **Textbook Content**: Qdrant vector database (unchanged, no migration needed)
- **User Preferences**: `programming_experience` field in Neon DB `users` table

### Schema Evolution

**Initial Schema** (v1):
```sql
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    programming_experience VARCHAR(50) NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
);
```

**Future Schema Changes** (examples):
- **v2**: Add `ros2_experience` ENUM field → Alembic migration adds column with default value
- **v3**: Add `hardware_access` BOOLEAN field → Alembic migration adds column with default `false`
- **v4**: Add `email_verified` BOOLEAN field → Alembic migration adds column with default `false`

**Migration Strategy**:
- Use **Alembic** (SQLAlchemy's migration tool) for all schema changes
- Generate migration scripts: `alembic revision --autogenerate -m "Add ros2_experience field"`
- Apply migrations: `alembic upgrade head` (run on deployment)
- Rollback capability: `alembic downgrade -1` (revert last migration)
- **Zero-Downtime Migrations**:
  1. Add new columns with default values (backward compatible)
  2. Deploy new backend code (uses new columns)
  3. Backfill old rows if needed
  4. Remove deprecated columns in future release (after 6-month deprecation period)

### Data Migration and Rollback

**Deployment Scenario**: Deploying auth feature to production

**Pre-Deployment**:
1. Create Neon DB database (if not exists)
2. Run Alembic migrations: `alembic upgrade head`
3. Verify schema: `SELECT * FROM users LIMIT 0;` (check columns exist)
4. Create test user: Manually insert via SQL to verify constraints

**Deployment**:
1. Deploy FastAPI backend to Hugging Face Spaces (build + push Docker image)
2. Set environment variables: `NEON_DATABASE_URL`, `JWT_SECRET_KEY`
3. Health check: `curl /health` to verify database connection
4. Test signup: Create test account via `/auth/signup`
5. Test signin: Authenticate test account via `/auth/signin`
6. Test chatbot: Send personalized query with auth token

**Rollback Plan** (if deployment fails):
1. Revert to previous Docker image on HF Spaces (instant rollback)
2. Database rollback: `alembic downgrade -1` (drop `users` table)
3. Frontend rollback: Disable auth UI (feature flag or revert commit)
4. **Data Preservation**: If users already signed up, do NOT drop table (keep data, disable feature in code)

**Data Retention**:
- User accounts: Retained indefinitely (no auto-deletion)
- JWT tokens: Self-expiring (7 days), no server-side storage
- Logs: Authentication failures logged for 30 days (security monitoring)

---

## 6. Operational Readiness

### Observability

**Logging**:
- **Auth Events**: Signup success/failure, signin attempts, token validation failures
- **Personalization Events**: User context extraction, score boosting applied
- **Errors**: Database connection failures, JWT signature errors, validation errors
- **Log Format**: Structured JSON logs with timestamp, event type, user_id (if available), error details
- **Log Levels**:
  - `INFO`: Successful signup, signin
  - `WARNING`: Invalid credentials, expired token
  - `ERROR`: Database connection failure, unexpected exceptions
- **PII Handling**: Do NOT log passwords, JWT tokens (only log token expiration time, user ID)

**Metrics** (future enhancement - not MVP):
- Auth success rate (signup, signin)
- Token validation latency (p50, p95, p99)
- Database query latency (p50, p95, p99)
- Personalization usage (% of queries with auth token)
- Programming experience distribution (Python, C++, None, Both)

**Traces** (future enhancement - not MVP):
- Distributed tracing with OpenTelemetry
- Trace auth flow: Request → JWT validation → DB query → Response
- Trace personalized chat: Request → JWT extraction → Qdrant query → Score boosting → LLM generation

### Alerting

**Critical Alerts** (future - not MVP):
- Database connection failure (> 5 failures in 5 minutes)
- Auth success rate < 95% (for valid credentials)
- JWT validation errors > 100/minute (possible attack or misconfiguration)

**On-Call Owners**:
- MVP: Single developer (manual monitoring)
- Production: Rotate on-call duty (future)

### Runbooks

#### Runbook 1: User Cannot Sign Up (409 Conflict Error)

**Symptoms**: User reports "Email already registered" but claims they never signed up

**Diagnosis**:
1. Query database: `SELECT * FROM users WHERE email = '<user_email>';`
2. Check if user exists with that email
3. Verify `created_at` timestamp (when account was created)

**Resolution**:
- If user exists: Instruct user to use "Sign In" instead (or password reset in future)
- If email is typo: User fixes email and tries again
- If malicious registration: Implement email verification in phase 2

**Prevention**: Add email verification to prevent fake signups

---

#### Runbook 2: Database Connection Failure (500 Error on Auth)

**Symptoms**: All auth endpoints return 500 errors, logs show "Connection refused" or "Database unavailable"

**Diagnosis**:
1. Check Neon DB status: Visit Neon dashboard, verify database is running
2. Check connection string: Verify `NEON_DATABASE_URL` environment variable is set correctly
3. Test connection: Run `psql $NEON_DATABASE_URL` to manually connect
4. Check firewall/IP whitelist: Verify HF Spaces IP is allowed (Neon should allow all by default)

**Resolution**:
- If Neon DB is down: Wait for Neon to restore (check status page)
- If connection string wrong: Update environment variable in HF Spaces settings, restart app
- If network issue: Contact Neon support or HF Spaces support

**Graceful Degradation**: Chatbot should still work in anonymous mode (verified by FR-016)

**Prevention**: Enable connection pooling with `pool_pre_ping=True` (auto-reconnect on stale connections)

---

#### Runbook 3: JWT Token Validation Failures (401 Errors)

**Symptoms**: Users report being logged out unexpectedly, all authenticated requests return 401

**Diagnosis**:
1. Check JWT secret: Verify `JWT_SECRET_KEY` environment variable is set and hasn't changed
2. Check token expiration: Decode JWT (jwt.io), verify `exp` claim hasn't passed
3. Check token signature: Verify token was signed with correct secret
4. Check backend logs: Look for "Invalid token signature" or "Token expired" errors

**Resolution**:
- If secret changed: Users must sign in again (all old tokens invalid)
- If tokens expired: Normal behavior (7-day expiration), users sign in again
- If signature mismatch: Redeploy backend with correct secret

**Prevention**: Never rotate JWT secret without user notification (invalidates all sessions)

---

### Deployment and Rollback Strategies

**Deployment Strategy**: Blue-Green Deployment (Simulated with HF Spaces)

1. **Build New Image**: Docker build with auth features
2. **Deploy to Staging** (future): Test in staging environment first
3. **Deploy to Production**: Push new image to HF Spaces
4. **Health Check**: Verify `/health` endpoint returns 200
5. **Smoke Test**: Create test account, sign in, send personalized query
6. **Monitor**: Watch logs for errors (first 10 minutes critical)
7. **Rollback Trigger**: If error rate > 5%, rollback immediately

**Rollback Strategy**: Instant Rollback (Docker Image Revert)

1. **Revert Image**: HF Spaces allows instant rollback to previous Docker image
2. **Database State**: Neon DB unchanged (auth table remains, no harm)
3. **Frontend State**: If auth UI deployed, revert frontend commit or disable feature flag
4. **User Impact**: Users with active sessions lose auth (minor inconvenience), chatbot works anonymously

**Feature Flags** (future enhancement):
- Environment variable `ENABLE_AUTH=true/false` to toggle auth feature without redeployment
- If `false`: Auth endpoints return 501 Not Implemented, chatbot works anonymously

### Backward Compatibility

**API Compatibility Promise**:
- Existing chatbot endpoints (`/chat`, `/stream-chat`) remain unchanged for anonymous users
- Adding optional `Authorization` header is backward compatible (clients without header work as before)
- No breaking changes to request/response formats

**Frontend Compatibility**:
- Users without auth UI update see no changes (anonymous mode works)
- Users with auth UI update see new features (signup/signin buttons)
- Gradual rollout: Deploy backend first, then frontend (backend handles both anonymous and authenticated)

**Database Compatibility**:
- New `users` table doesn't affect existing Qdrant data
- No changes to Cohere API usage or configuration
- Neon DB and Qdrant are independent (no cross-database dependencies)

---

## 7. Risk Analysis and Mitigation

### Top 3 Technical Risks

#### Risk 1: Neon DB Connection Failures During Peak Load

**Likelihood**: Medium
**Impact**: High (auth completely unavailable)
**Blast Radius**: All auth endpoints (signup, signin, profile); chatbot anonymous mode unaffected

**Mitigation**:
- ✅ **Connection Pooling**: SQLAlchemy pool with 5 min connections, 10 max overflow (handles bursts)
- ✅ **Health Check**: `pool_pre_ping=True` validates connections before use (handles stale connections)
- ✅ **Graceful Degradation**: Chatbot falls back to anonymous mode if user query fails (FR-016)
- ✅ **Monitoring**: Log database connection errors, alert if > 5 failures in 5 minutes
- ⚠️ **Future**: Implement read replicas for high availability (Neon Pro feature)

**Kill Switch**: Environment variable `ENABLE_AUTH=false` disables auth without redeployment

**Guardrails**: Database query timeout (5s), retry with exponential backoff (3 attempts max)

---

#### Risk 2: JWT Secret Leak (Security Breach)

**Likelihood**: Low
**Impact**: Critical (attacker can forge tokens, impersonate any user)
**Blast Radius**: All authenticated users (attacker gains full access to any account)

**Mitigation**:
- ✅ **Secret Management**: Store `JWT_SECRET_KEY` in HF Spaces secrets (encrypted at rest, not in code)
- ✅ **Secret Rotation**: If leak suspected, rotate secret immediately (invalidates all tokens, users re-signin)
- ✅ **Strong Secret**: Generate 256-bit random secret (`openssl rand -hex 32`), not human-created
- ✅ **Monitoring**: Log unusual token validation patterns (e.g., tokens with future exp claims)
- ⚠️ **Future**: Implement token blacklist (requires Redis) for immediate revocation

**Incident Response**:
1. Rotate JWT secret (set new `JWT_SECRET_KEY` environment variable)
2. Force all users to sign in again (all old tokens invalid)
3. Review logs for suspicious activity (tokens used by attacker)
4. Notify affected users if data accessed

**Prevention**: Never commit secrets to Git (use `.env` files, `.gitignore` them)

---

#### Risk 3: Personalization Algorithm Doesn't Improve User Experience

**Likelihood**: Medium
**Impact**: Medium (feature provides no value, wasted effort)
**Blast Radius**: Authenticated users only (anonymous users unaffected)

**Mitigation**:
- ✅ **Gradual Rollout**: Deploy auth, collect feedback before marketing feature
- ✅ **A/B Testing** (future): Compare satisfaction of personalized vs anonymous users
- ✅ **Metrics**: Track programming experience distribution, verify content diversity
- ✅ **User Feedback**: Add "Was this helpful?" button to chatbot responses
- ✅ **Tunable Parameters**: Score boost factors (1.3x) are configurable, adjust based on feedback
- ⚠️ **Fallback**: If personalization hurts UX, disable with feature flag

**Success Criteria** (from spec.md SC-002, SC-003):
- Python users receive Python examples in 80%+ of technical queries
- C++ users receive C++ examples in 80%+ of technical queries

**Kill Switch**: Set all boost factors to 1.0 (disable personalization, keep auth for future features)

---

### Operational Risks

#### Risk 4: Users Forget Passwords (No Reset Mechanism in MVP)

**Likelihood**: High
**Impact**: Low (users create new account with different email)
**Mitigation**: Document limitation, prioritize password reset in phase 2

#### Risk 5: Email Harvesting via Public Signup Endpoint

**Likelihood**: Medium
**Impact**: Low (spam risk, no financial impact)
**Mitigation**: Add rate limiting (5 signups/IP/hour) in phase 2, monitor for abuse

#### Risk 6: XSS Attack via Malicious User Input

**Likelihood**: Low (React auto-escapes, Docusaurus has CSP)
**Impact**: Medium (token theft via localStorage)
**Mitigation**: Trust React/Docusaurus protections, audit third-party components, consider httpOnly cookies in phase 2

---

## 8. Evaluation and Validation

### Definition of Done (DoD)

**Feature is complete when ALL criteria met**:

1. ✅ **Unit Tests Pass**:
   - Password validation logic (strength rules)
   - JWT generation and validation (exp, signature)
   - Score boosting algorithm (1.3x multiplier)
   - SQLAlchemy models (CRUD operations)

2. ✅ **Integration Tests Pass**:
   - Signup → Signin → Authenticated chat (end-to-end flow)
   - Profile update → Next chat reflects new preference
   - Anonymous chat → Signup → Same question returns personalized response
   - Expired token → 401 error → Prompt to re-signin

3. ✅ **API Contract Validation**:
   - All endpoints match OpenAPI spec (request/response schemas)
   - Error responses follow consistent format (`{"detail": "..."}`)
   - CORS headers present (allow production origin)

4. ✅ **Security Scans Pass**:
   - `pip-audit` shows no high/critical vulnerabilities
   - No passwords in logs (manual log review)
   - No JWT secrets in code (grep for "JWT_SECRET")
   - SQL injection test (attempt malicious input, verify ORM blocks it)

5. ✅ **Performance Targets Met**:
   - Signup: p95 < 2s
   - Signin: p95 < 2s
   - Token validation: p99 < 100ms
   - Personalized chat: p95 < 5s (no degradation from current)

6. ✅ **Acceptance Scenarios Pass**:
   - All 6 user stories from spec.md tested manually
   - Python user receives Python examples first
   - C++ user receives C++ examples first
   - Anonymous user can access chatbot without signup

7. ✅ **Documentation Complete**:
   - API contracts documented (this file)
   - Database schema documented (data-model.md)
   - Deployment runbook written (section 6 of this plan)
   - ADRs created for key decisions (next section)

8. ✅ **Deployment Successful**:
   - Deployed to HF Spaces (production)
   - Health check returns 200 OK
   - Test signup/signin works in production
   - Frontend deployed with auth UI

### Output Validation

**Automated Checks** (CI/CD pipeline - future):
```bash
# Unit tests
pytest tests/test_auth.py --cov=app.auth

# Integration tests
pytest tests/test_integration.py --integration

# Security scan
pip-audit --strict

# API contract validation (OpenAPI spec)
pytest tests/test_api_contracts.py

# Performance tests (load testing)
locust -f tests/locustfile.py --headless -u 50 -r 10 --run-time 1m
```

**Manual Checks** (pre-deployment):
- [ ] Create account with valid email/password
- [ ] Attempt duplicate email signup (verify 409 error)
- [ ] Sign in with correct credentials
- [ ] Sign in with wrong password (verify 401 error)
- [ ] Update programming experience (Python → C++)
- [ ] Ask same question twice (verify different examples)
- [ ] Sign out, ask question anonymously (verify generic response)
- [ ] Check logs for any passwords or JWT tokens

**Success Metrics** (post-deployment):
- Signup success rate: > 99% (valid inputs)
- Signin success rate: > 99% (correct credentials)
- Chatbot personalization accuracy: > 80% (Python users get Python, C++ users get C++)
- Zero security incidents in first month
- User satisfaction: > 4/5 stars (feedback survey - future)

---

## Architectural Decision Records (ADRs)

### ADR-001: Custom JWT Authentication

**Status**: Accepted
**Date**: 2025-12-27
**Deciders**: Project Team

**Context**:
We need authentication for user signup/signin to enable personalized content recommendations. Better-Auth was originally specified but is designed for Node.js/Next.js, not Python/FastAPI.

**Decision**:
Implement custom JWT authentication using PyJWT and bcrypt instead of Better-Auth or FastAPI-Users.

**Rationale**:
- Better-Auth has no official Python support (ecosystem mismatch)
- FastAPI-Users is overkill for simple use case (email/password + one profile field)
- Custom implementation is lightweight (~200 LOC), fully controlled, easy to debug
- PyJWT and bcrypt are battle-tested, industry-standard libraries
- Aligns with "smallest viable change" principle from constitution

**Consequences**:
- ✅ Full control over auth logic (optimize for personalization use case)
- ✅ No vendor lock-in or heavy dependencies
- ✅ Simple implementation (~200 LOC vs 1000+ with FastAPI-Users)
- ❌ Missing advanced features (OAuth, MFA) - acceptable for MVP, can add later
- ❌ Manual security maintenance - mitigated by using standard libraries

**Related Decisions**: ADR-002 (JWT tokens), ADR-005 (localStorage)

---

### ADR-002: JWT Tokens with 7-Day Expiration

**Status**: Accepted
**Date**: 2025-12-27
**Deciders**: Project Team

**Context**:
We need session management for authenticated users. Options include session-based auth (server-side state) or JWT tokens (stateless).

**Decision**:
Use stateless JWT tokens with 7-day expiration instead of session-based auth.

**Rationale**:
- Current backend is deployed on Hugging Face Spaces (serverless, stateless environment)
- No Redis or session store infrastructure available
- Educational textbook has low security risk (no financial data)
- JWT tokens scale horizontally without session synchronization
- 7-day expiration balances convenience (stay logged in) and security (forced re-auth)

**Consequences**:
- ✅ Scales with serverless deployment (no state on backend)
- ✅ No additional infrastructure (Redis, session store)
- ✅ Works with existing HF Spaces deployment
- ❌ Cannot revoke tokens before expiration - mitigated by short expiration
- ❌ Slightly larger token size - negligible network overhead

**Related Decisions**: ADR-001 (custom JWT auth), ADR-005 (localStorage)

---

### ADR-003: Relevance Score Boosting for Personalization

**Status**: Accepted
**Date**: 2025-12-27
**Deciders**: Project Team

**Context**:
We need to personalize chatbot responses based on user's programming experience (Python, C++, None, Both). Options include pre-filtering, post-search LLM re-ranking, or score boosting.

**Decision**:
Implement relevance score boosting (multiply similarity scores by 1.3x for language match) instead of pre-filtering or LLM re-ranking.

**Rationale**:
- Pre-filtering is too aggressive (excludes valuable content, reduces diversity)
- LLM re-ranking adds 2-3s latency and API cost (unacceptable per NFR-007: 5s response time)
- Score boosting provides "soft" personalization (Python users see Python first, but C++ isn't hidden)
- Implementation is simple (~30 LOC modification to existing query logic)
- Falls back gracefully if metadata is missing

**Consequences**:
- ✅ Minimal latency overhead (<50ms for score multiplication)
- ✅ Preserves content diversity (doesn't exclude non-matching languages)
- ✅ Simple implementation (30 LOC)
- ❌ Less aggressive than pre-filtering - acceptable trade-off for UX
- ❌ Requires language metadata in Qdrant payloads - likely already present

**Related Decisions**: None (personalization algorithm is independent)

---

### ADR-004: SQLAlchemy ORM with Neon DB

**Status**: Accepted
**Date**: 2025-12-27
**Deciders**: Project Team

**Context**:
We need an ORM for database interactions with Neon DB (PostgreSQL). Options include Prisma (TypeScript-first), raw SQL with psycopg2, or SQLAlchemy ORM.

**Decision**:
Use SQLAlchemy ORM with Neon DB instead of Prisma or raw SQL.

**Rationale**:
- SQLAlchemy is the de facto standard ORM for Python web applications
- FastAPI documentation uses SQLAlchemy in all official examples
- Provides connection pooling out-of-box (required by NFR-008)
- Type hints with Pydantic models ensure type safety
- Alembic (SQLAlchemy companion) handles migrations for future schema changes
- Prisma has limited Python support (designed for Node.js)

**Consequences**:
- ✅ Python ecosystem standard (mature, well-documented)
- ✅ Connection pooling built-in (min 5 connections per NFR-008)
- ✅ Migration support with Alembic
- ❌ Slightly more verbose than Prisma - acceptable trade-off
- ❌ Learning curve for complex queries - not an issue for simple CRUD operations

**Related Decisions**: ADR-001 (auth implementation uses SQLAlchemy models)

---

### ADR-005: localStorage for JWT Token Storage

**Status**: Accepted
**Date**: 2025-12-27
**Deciders**: Project Team

**Context**:
We need to store JWT tokens on the client side. Options include httpOnly cookies (more secure, protected from XSS) or localStorage (simpler, easier to debug).

**Decision**:
Store JWT tokens in localStorage instead of httpOnly cookies.

**Rationale**:
- Educational textbook has low security risk (no financial data, no sensitive PII)
- XSS risk is already mitigated by React's default escaping and Docusaurus CSP
- localStorage is standard pattern for SPAs with JWT auth (Auth0, Firebase use this)
- Simplifies CORS configuration (no need for `credentials: true`)
- Easier to implement and debug (inspect localStorage in DevTools)

**Consequences**:
- ✅ Simple implementation (standard SPA pattern)
- ✅ Works with existing CORS setup
- ✅ Easy to debug (DevTools)
- ❌ Vulnerable to XSS attacks - mitigated by React auto-escaping and CSP
- ❌ Less secure than httpOnly cookies - acceptable for low-risk app

**Related Decisions**: ADR-002 (JWT tokens), ADR-001 (custom auth)

**Future Consideration**: Upgrade to httpOnly cookies in phase 2 if security requirements increase

---

### ADR-006: Optional Authentication for Chatbot Access

**Status**: Accepted
**Date**: 2025-12-27
**Deciders**: Project Team

**Context**:
We need to decide whether users must sign in to access the chatbot or if anonymous access is allowed. Options include required auth (all users must sign in), optional auth (signup for personalization benefits), or freemium (limited anonymous access).

**Decision**:
Make authentication optional - anonymous users can access chatbot with generic responses, authenticated users get personalized responses.

**Rationale**:
- Constitution principle: "User experience drives content ranking" - forced auth degrades experience
- Educational content should be accessible to all (open education philosophy)
- Users can evaluate chatbot quality before committing to signup
- Personalization serves as incentive (carrot) rather than forced gate (stick)
- Aligns with Docusaurus philosophy (static site, freely accessible documentation)

**Consequences**:
- ✅ Low friction (users try chatbot before signup)
- ✅ Gradual trust building (experience value, then commit)
- ✅ Aligns with open education mission
- ❌ Some anonymous usage (no user data for all users) - acceptable trade-off
- ❌ Lower signup rate initially - compensated by better long-term retention

**Related Decisions**: ADR-003 (personalization benefits authenticated users)

---

## Implementation Phases

### Phase 1: Backend Foundation (Priority: P1)

**Estimated Effort**: 2 days
**Dependencies**: Neon DB account, HF Spaces deployment access

**Tasks**:
1. Set up Neon DB database and `users` table (1 hour)
2. Create SQLAlchemy models and database connection (2 hours)
3. Implement auth routes: `/auth/signup`, `/auth/signin`, `/auth/me` (4 hours)
4. Implement JWT generation and validation middleware (3 hours)
5. Add password hashing with bcrypt (1 hour)
6. Write unit tests for auth logic (2 hours)
7. Deploy to HF Spaces with environment variables (1 hour)

**Acceptance Criteria**:
- `POST /auth/signup` creates user and returns JWT token
- `POST /auth/signin` authenticates user and returns JWT token
- `GET /auth/me` returns user profile for valid token
- Unit tests pass (password validation, JWT generation)

---

### Phase 2: Personalization Engine (Priority: P1)

**Estimated Effort**: 1 day
**Dependencies**: Phase 1 complete

**Tasks**:
1. Modify `/chat` and `/stream-chat` to extract user context from JWT (2 hours)
2. Implement score boosting algorithm for Qdrant results (3 hours)
3. Add personalization metadata to responses (`personalized: true/false`) (1 hour)
4. Write integration tests (authenticated vs anonymous queries) (2 hours)

**Acceptance Criteria**:
- Authenticated Python user receives Python examples first
- Authenticated C++ user receives C++ examples first
- Anonymous user receives generic responses
- Integration tests pass

---

### Phase 3: Frontend Auth UI (Priority: P2)

**Estimated Effort**: 2 days
**Dependencies**: Phase 1 complete

**Tasks**:
1. Create SigninModal and SignupModal components (4 hours)
2. Modify header to show signin/signup button (anonymous) or profile dropdown (authenticated) (2 hours)
3. Update API client to include `Authorization` header (1 hour)
4. Implement JWT token storage in localStorage (1 hour)
5. Create ProfileSettingsModal for updating programming experience (2 hours)
6. Add anonymous user banner promoting signup (1 hour)
7. Test end-to-end flow (signup → signin → personalized chat) (2 hours)

**Acceptance Criteria**:
- User can sign up, sign in, and sign out via UI
- Authenticated user sees profile dropdown in header
- Anonymous user sees signup banner in chatbot
- Personalized responses show different examples than generic responses

---

### Phase 4: Testing & Deployment (Priority: P1)

**Estimated Effort**: 1 day
**Dependencies**: Phases 1, 2, 3 complete

**Tasks**:
1. Run full integration test suite (1 hour)
2. Perform security audit (SQL injection, XSS, password in logs) (2 hours)
3. Load testing (50 concurrent users) (1 hour)
4. Deploy frontend to Vercel (1 hour)
5. Smoke test in production (signup, signin, chat) (1 hour)
6. Monitor logs for errors (first hour after deployment)
7. Create deployment runbook and ADRs (1 hour)

**Acceptance Criteria**:
- All tests pass (unit, integration, security)
- Performance targets met (signup < 2s, chat < 5s)
- Production deployment successful
- No errors in logs (first hour)

---

## Summary of Key Files to Create/Modify

### Backend (FastAPI)

**New Files**:
- `rag-backend/chatbot/auth.py` - Auth routes and JWT logic (~200 LOC)
- `rag-backend/chatbot/models.py` - SQLAlchemy User model (~50 LOC)
- `rag-backend/chatbot/schemas.py` - Pydantic request/response models (~100 LOC)
- `rag-backend/chatbot/database.py` - Database connection and session management (~30 LOC)
- `rag-backend/chatbot/middleware.py` - JWT validation middleware (~50 LOC)
- `rag-backend/chatbot/alembic/` - Database migration scripts (generated by Alembic)

**Modified Files**:
- `rag-backend/chatbot/app.py` - Add auth routes, modify `/chat` and `/stream-chat` for personalization (~100 LOC changes)
- `rag-backend/chatbot/config.py` - Add Neon DB connection string and JWT secret config (~20 LOC)
- `rag-backend/chatbot/requirements.txt` - Add dependencies: `sqlalchemy`, `psycopg2-binary`, `pyjwt`, `bcrypt`, `alembic`

### Frontend (React + Docusaurus)

**New Files**:
- `my-website/src/components/Auth/SigninModal.tsx` - Signin form modal (~150 LOC)
- `my-website/src/components/Auth/SignupModal.tsx` - Signup form modal (~200 LOC)
- `my-website/src/components/Auth/ProfileSettingsModal.tsx` - Profile settings modal (~150 LOC)
- `my-website/src/components/Auth/AuthContext.tsx` - React context for auth state (~100 LOC)
- `my-website/src/components/Auth/AuthButton.tsx` - Header button (signin/profile) (~100 LOC)
- `my-website/src/components/Auth/styles.module.css` - Auth UI styles (~50 LOC)

**Modified Files**:
- `my-website/src/components/ChatBot/api.ts` - Add auth methods (`signup`, `signin`, `getProfile`) and include `Authorization` header (~100 LOC changes)
- `my-website/src/theme/Layout/index.tsx` - Add AuthContext provider and AuthButton to header (~30 LOC)
- `my-website/src/components/ChatBot/ChatWindow.tsx` - Add anonymous user banner (~20 LOC)

### Documentation

**New Files**:
- `specs/002-better-auth-signup-signin/plan.md` - This file (implementation plan)
- `history/adr/001-custom-jwt-authentication.md` - ADR for auth library choice
- `history/adr/002-jwt-tokens-7day-expiration.md` - ADR for session management
- `history/adr/003-relevance-score-boosting.md` - ADR for personalization algorithm
- `history/adr/004-sqlalchemy-orm-neon-db.md` - ADR for ORM choice
- `history/adr/005-localstorage-jwt-storage.md` - ADR for token storage
- `history/adr/006-optional-authentication.md` - ADR for auth requirement

**Modified Files**:
- `README.md` - Add auth setup instructions (Neon DB connection, JWT secret)

---

## Total Implementation Estimate

- **Backend**: 2 days
- **Personalization**: 1 day
- **Frontend**: 2 days
- **Testing & Deployment**: 1 day

**Total**: 6 days (with buffer for unexpected issues)

**Critical Path**: Backend → Personalization → Frontend → Testing (sequential)

**Parallelization Opportunities**:
- Frontend auth UI can start after backend auth routes are complete (Phase 3 can overlap with Phase 2)
- Unit tests can be written in parallel with implementation

---

## Next Steps

1. **Create Neon DB database** (5 minutes)
   - Sign up at neon.tech
   - Create database: `humanoid-robotics-textbook`
   - Copy connection string to environment variable

2. **Generate JWT secret** (1 minute)
   - Run: `openssl rand -hex 32`
   - Store in HF Spaces secrets: `JWT_SECRET_KEY`

3. **Run `/sp.tasks`** to generate detailed implementation tasks

4. **Create feature branch**: `git checkout -b 002-better-auth-signup-signin`

5. **Begin Phase 1**: Set up backend foundation

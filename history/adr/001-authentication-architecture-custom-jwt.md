# ADR-001: Authentication Architecture - Custom JWT with localStorage

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-27
- **Feature:** better-auth-signup-signin
- **Context:** The textbook application needs user authentication to enable personalized content recommendations based on programming experience (Python, C++, None, Both). The system is deployed on serverless infrastructure (Hugging Face Spaces) with a React frontend on Vercel. Security requirements are moderate (educational content, no financial data). Better-Auth was initially specified but lacks Python/FastAPI support.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? YES - defines entire auth system
     2) Alternatives: Multiple viable options considered with tradeoffs? YES - Better-Auth, FastAPI-Users, session-based, httpOnly cookies
     3) Scope: Cross-cutting concern (not an isolated detail)? YES - affects backend API, frontend UI, security model
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Implement a custom JWT-based authentication system using:

- **Backend Auth Library**: Custom implementation with PyJWT + bcrypt (not Better-Auth or FastAPI-Users)
- **Session Mechanism**: Stateless JWT tokens with 7-day expiration (not server-side sessions)
- **Token Storage**: localStorage in browser (not httpOnly cookies or sessionStorage)
- **Token Payload**: User ID, email, programming_experience (embedded for personalization)
- **Password Hashing**: bcrypt with 10 rounds minimum (OWASP recommendation)
- **Token Algorithm**: HS256 (HMAC with SHA-256)

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

- **Lightweight Implementation**: ~200 LOC for complete auth system vs 1000+ with FastAPI-Users
- **Full Control**: Optimize JWT payload for personalization (embed programming_experience)
- **Serverless-Compatible**: No session store (Redis) needed; works with stateless HF Spaces
- **Horizontal Scalability**: JWT tokens scale without session synchronization across instances
- **Developer Experience**: Easy to debug (inspect JWT at jwt.io, localStorage in DevTools)
- **No Vendor Lock-In**: PyJWT and bcrypt are standard libraries, not proprietary frameworks
- **CORS Simplicity**: localStorage doesn't require `credentials: true` or CSRF protection
- **Standard Pattern**: Auth0, Firebase, and other SPAs use JWT + localStorage

<!-- Example: Integrated tooling, excellent DX, fast deploys, strong TypeScript support -->

### Negative

- **Manual Security Maintenance**: Responsible for JWT validation, password hashing, token expiration logic (no framework abstractions)
- **Missing Advanced Features**: No built-in OAuth, MFA, email verification, password reset (must implement separately if needed)
- **Token Revocation Limitation**: Cannot invalidate tokens before expiration (7-day window if secret leaked)
- **XSS Vulnerability**: localStorage tokens accessible to malicious JavaScript (mitigated by React auto-escaping and Docusaurus CSP)
- **Token Size**: JWT tokens (~500 bytes) larger than session IDs (~50 bytes) - negligible network overhead
- **Re-Inventing Wheels**: Implementing auth logic that FastAPI-Users provides out-of-box

<!-- Example: Vendor lock-in to Vercel, framework coupling, learning curve -->

## Alternatives Considered

### Alternative 1: Better-Auth (Node.js Auth Library)
- **Components**: Better-Auth framework, Node.js backend, httpOnly cookies
- **Why Rejected**: Better-Auth is designed for Node.js/Next.js; no official Python/FastAPI support. Would require rewriting backend in Node.js or using separate auth service (architectural complexity).

### Alternative 2: FastAPI-Users (Python Auth Framework)
- **Components**: FastAPI-Users library, database models, OAuth support, email verification
- **Why Rejected**: Heavy abstraction (1000+ LOC dependency) for simple use case (email/password + one profile field). Opinionated structure conflicts with existing backend architecture. Overkill for MVP without OAuth/MFA requirements.

### Alternative 3: Session-Based Auth with Redis
- **Components**: Server-side sessions, Redis session store, session cookies
- **Why Rejected**: Requires Redis infrastructure (cost, complexity). Not compatible with serverless HF Spaces deployment. Adds state management complexity (session synchronization, sticky sessions). Unnecessary for low-security educational content.

### Alternative 4: httpOnly Cookies for Token Storage
- **Components**: Same JWT system, but store tokens in httpOnly cookies instead of localStorage
- **Why Rejected**: Requires CORS credentials configuration (`credentials: true`), CSRF protection (additional complexity). Harder to debug (cannot inspect in DevTools). Educational content has low security risk (no financial data) - localStorage XSS vulnerability acceptable given React protections.

### Alternative 5: JWT with Token Blacklist (Hybrid)
- **Components**: Stateless JWT + Redis blacklist for revoked tokens
- **Why Rejected**: Adds infrastructure complexity (Redis) for minimal benefit. Educational textbook use case doesn't require immediate revocation. Short 7-day expiration mitigates leaked token risk. Premature optimization.

<!-- Group alternatives by cluster:
     Alternative Stack A: Remix + styled-components + Cloudflare
     Alternative Stack B: Vite + vanilla CSS + AWS Amplify
     Why rejected: Less integrated, more setup complexity
-->

## References

- Feature Spec: [specs/002-better-auth-signup-signin/spec.md](../../specs/002-better-auth-signup-signin/spec.md)
- Implementation Plan: [specs/002-better-auth-signup-signin/plan.md](../../specs/002-better-auth-signup-signin/plan.md#decision-1-use-custom-jwt-auth-with-fastapi-not-better-auth)
- Related ADRs:
  - ADR-004 (SQLAlchemy for user account storage)
  - ADR-003 (Personalization algorithm uses JWT context)
- Evaluator Evidence: [history/prompts/better-auth-signup-signin/0001-implementation-plan-better-auth-signup-signin.plan.prompt.md](../prompts/better-auth-signup-signin/0001-implementation-plan-better-auth-signup-signin.plan.prompt.md)

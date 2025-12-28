# ADR-004: Optional Authentication Model

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-27
- **Feature:** better-auth-signup-signin
- **Context:** The textbook application provides a chatbot for answering robotics questions. We're adding authentication to enable personalized responses based on user's programming experience. Key question: Should users be required to sign in before accessing the chatbot, or should authentication be optional with personalization benefits? Constitution principle states "User experience drives content ranking" and the project is educational content (open education philosophy).

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? YES - defines product strategy and user acquisition funnel
     2) Alternatives: Multiple viable options considered with tradeoffs? YES - required auth, optional auth, freemium model
     3) Scope: Cross-cutting concern (not an isolated detail)? YES - affects UX, backend endpoints, frontend flows, business metrics
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Implement **optional authentication** model:

- **Anonymous Access**: Users can access all chatbot functionality without signing up (no forced signin wall)
- **Personalization Incentive**: Authenticated users receive personalized responses (programming language-specific examples)
- **Anonymous Experience**: Generic responses for anonymous users (balanced Python/C++ examples)
- **Signup Promotion**: Display informational banner to anonymous users: "Sign up for personalized responses based on your programming experience"
- **No Rate Limiting (MVP)**: Anonymous users have unlimited queries (monitor for abuse, add rate limiting if needed)
- **Graceful Degradation**: If authentication system fails, chatbot continues working anonymously (no service disruption)

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

- **Low Friction Onboarding**: Users experience chatbot value before committing to signup (reduces abandonment)
- **Aligns with Educational Mission**: Open education philosophy - knowledge should be accessible to all
- **Gradual Trust Building**: Users evaluate quality first, then provide personal information
- **Constitution Compliance**: "User experience drives content ranking" - forced auth degrades experience
- **Docusaurus Philosophy**: Static site with freely accessible documentation (no paywalls)
- **Valuable Anonymous Feedback**: Anonymous usage provides testing data and product insights
- **Higher Long-Term Retention**: Users who sign up after experiencing value are more committed (vs forced signup)
- **No Rate Limiting Complexity**: MVP avoids IP tracking, CAPTCHA, abuse detection infrastructure

<!-- Example: Integrated tooling, excellent DX, fast deploys, strong TypeScript support -->

### Negative

- **Lower Signup Conversion Rate**: Some users satisfied with generic responses may never sign up
- **No User Tracking for Anonymous**: Cannot analyze anonymous user behavior or personalize future visits
- **Potential for Abuse**: Unlimited anonymous queries enable spam/bots (requires monitoring)
- **Reduced Personalization Reach**: Only authenticated users benefit from feature (limits impact)
- **Incomplete User Data**: Cannot correlate all chatbot usage with user profiles
- **Delayed Value Capture**: Users experience value before providing email (email list growth slower)

<!-- Example: Vendor lock-in to Vercel, framework coupling, learning curve -->

## Alternatives Considered

### Alternative 1: Required Authentication (Force Signin Before Chatbot Access)
- **Components**: Signin wall, redirect to auth page on chatbot click, no anonymous access
- **Why Rejected**: High friction deters casual users. Reduces engagement (users leave before experiencing value). Violates constitution principle "User experience drives content ranking". Not aligned with open education philosophy. Users cannot evaluate quality before committing personal information.

### Alternative 2: Freemium Model (Limited Anonymous Access, Unlimited for Authenticated)
- **Components**: Rate limiting (e.g., 10 queries/day for anonymous), IP tracking, CAPTCHA for abuse prevention, upgrade prompt after limit
- **Why Rejected**: Adds complexity (rate limiting infrastructure, IP tracking, abuse detection). Frustrates users with artificial limits. Overkill for educational content (no monetization plan). Requires maintenance burden. May violate GDPR (IP tracking without consent).

### Alternative 3: Soft Signup Prompt (Optional Auth with Aggressive Prompts)
- **Components**: Optional auth, but frequent modal prompts to sign up (e.g., after every 3rd query)
- **Why Rejected**: Annoying UX (interrupts user flow). Feels like forced auth disguised as optional. Degrades user experience. Not aligned with educational mission (prioritize learning, not conversion). Users may abandon due to prompt fatigue.

### Alternative 4: Progressive Enhancement (Anonymous → Guest Account → Full Account)
- **Components**: Anonymous usage, automatic guest account creation (UUID in localStorage), optional email upgrade
- **Why Rejected**: Added complexity (guest account logic, migration path). Confusing UX (users don't understand guest vs full account). No clear benefit over simple optional auth. Over-engineered for MVP.

### Alternative 5: Invitation-Only Beta (Closed Access During MVP)
- **Components**: Waitlist, manual approval, email invitations, private beta testing
- **Why Rejected**: Limits feedback (small user base). Slow iteration (can't test at scale). Not aligned with open education philosophy. Artificial scarcity inappropriate for educational content.

<!-- Group alternatives by cluster:
     Alternative Stack A: Remix + styled-components + Cloudflare
     Alternative Stack B: Vite + vanilla CSS + AWS Amplify
     Why rejected: Less integrated, more setup complexity
-->

## References

- Feature Spec: [specs/002-better-auth-signup-signin/spec.md](../../specs/002-better-auth-signup-signin/spec.md#user-story-3---anonymous-access-with-limited-personalization-priority-p1)
- Implementation Plan: [specs/002-better-auth-signup-signin/plan.md](../../specs/002-better-auth-signup-signin/plan.md#decision-6-optional-authentication-not-required-for-chatbot-access)
- Constitution: [.specify/memory/constitution.md](../../.specify/memory/constitution.md) (Principle: "User experience drives content ranking")
- Related ADRs:
  - ADR-001 (JWT auth system supports optional authentication)
  - ADR-003 (Personalization benefits authenticated users)
- Evaluator Evidence: [history/prompts/better-auth-signup-signin/0001-implementation-plan-better-auth-signup-signin.plan.prompt.md](../prompts/better-auth-signup-signin/0001-implementation-plan-better-auth-signup-signin.plan.prompt.md)

## Implementation Notes

**Backend Changes**:
- Chatbot endpoints (`/chat`, `/stream-chat`) accept optional `Authorization` header
- If header present and valid → extract user context, apply personalization
- If header missing/invalid → anonymous mode, generic responses
- Return metadata: `personalized: true/false` (frontend can display badge)

**Frontend Changes**:
- Header: Show "Sign In / Sign Up" button for anonymous users
- Chatbot: Display banner for anonymous users: "Sign up for personalized responses"
- No redirect to signin page (users stay on current page)
- Subtle promotion (not aggressive modal interruptions)

**Monitoring**:
- Track signup conversion rate (anonymous → authenticated)
- Track abuse patterns (high-volume anonymous queries from single IP)
- Track user satisfaction (feedback survey: anonymous vs authenticated)

**Kill Switch**:
- Environment variable `ENABLE_ANONYMOUS_ACCESS=true/false`
- If abuse detected, toggle to require authentication without redeployment

# Feature Specification: Better-Auth Signup & Signin with User Personalization

**Feature Branch**: `002-better-auth-signup-signin`
**Created**: 2025-12-27
**Status**: Draft
**Input**: Project Constitution: "Implement Signup & Signin using Better-Auth, collect software & hardware background from users at signup, use background info to personalize content recommendations from Qdrant database"

## User Scenarios & Testing *(mandatory)*

<!--
  IMPORTANT: User stories should be PRIORITIZED as user journeys ordered by importance.
  Each user story/journey must be INDEPENDENTLY TESTABLE - meaning if you implement just ONE of them,
  you should still have a viable MVP (Minimum Viable Product) that delivers value.

  Assign priorities (P1, P2, P3, etc.) to each story, where P1 is the most critical.
  Think of each story as a standalone slice of functionality that can be:
  - Developed independently
  - Tested independently
  - Deployed independently
  - Demonstrated to users independently
-->

### User Story 1 - Basic Signup & Signin (Priority: P1)

As a **new user**, when I visit the textbook website, I can create an account using my email and password, and subsequently sign in to access my personalized experience.

**Why this priority**: This is the foundation for all personalization features. Without authentication, we cannot identify users or store their preferences. This is the minimum viable product that enables all subsequent features.

**Independent Test**: Can be fully tested by accessing the website, clicking "Sign Up", entering email/password, submitting the form, receiving a success confirmation, signing out, and signing back in with the same credentials.

**Acceptance Scenarios**:

1. **Given** I am a new user on the homepage, **When** I click the "Sign Up" button and enter valid email/password, **Then** my account is created and I am automatically signed in
2. **Given** I have an existing account, **When** I click "Sign In" and enter my credentials, **Then** I am authenticated and see my profile name in the header
3. **Given** I am signed in, **When** I click "Sign Out", **Then** I am logged out and returned to the public homepage
4. **Given** I try to sign up with an already-registered email, **When** I submit the form, **Then** I see an error message "Email already registered"
5. **Given** I enter an invalid password during signin, **When** I submit the form, **Then** I see an error message "Invalid credentials"

---

### User Story 2 - Programming Experience Collection (Priority: P1)

As a **new user during signup**, I am asked about my programming experience (Python/C++/None), which is stored in my profile to enable personalized content recommendations.

**Why this priority**: Programming experience is the single most important personalization factor (per user's selection). Collecting this during signup ensures we have the data needed for content ranking immediately. This is P1 because personalization is a core value proposition stated in the constitution.

**Independent Test**: Can be tested by completing the signup flow with different programming experience selections, then verifying that subsequent chatbot responses include content appropriate to that experience level.

**Acceptance Scenarios**:

1. **Given** I am signing up, **When** I see the signup form, **Then** I am presented with a "Programming Experience" dropdown with options: "None", "Python", "C++", "Both Python and C++"
2. **Given** I select "None" for programming experience, **When** I ask the chatbot for code examples, **Then** responses prioritize explanatory text and pseudocode over complex syntax
3. **Given** I select "Python" as my programming experience, **When** I ask about ROS2 code, **Then** chatbot responses include Python (rclpy) examples first
4. **Given** I select "C++" as my programming experience, **When** I ask about ROS2 code, **Then** chatbot responses include C++ (rclcpp) examples first
5. **Given** I select "Both Python and C++", **When** I ask for code examples, **Then** chatbot shows both implementations side-by-side

---

### User Story 3 - Anonymous Access with Limited Personalization (Priority: P1)

As a **visitor who prefers not to sign up**, I can still use the chatbot and read content, but I receive generic (non-personalized) responses.

**Why this priority**: Per user selection, signin should be optional. This is P1 because forcing authentication creates friction and reduces engagement. Anonymous access is the fallback that ensures the textbook remains accessible to all.

**Independent Test**: Can be tested by accessing the website without signing in, asking chatbot questions, and verifying that responses do not include personalization indicators (e.g., no "Based on your Python experience..." messages).

**Acceptance Scenarios**:

1. **Given** I am an anonymous user, **When** I access the website, **Then** I can read all textbook content and use the chatbot without restriction
2. **Given** I am an anonymous user, **When** I ask a chatbot question, **Then** I receive generic responses without programming-language-specific bias
3. **Given** I am an anonymous user, **When** I see the header, **Then** I see a "Sign In / Sign Up" button inviting me to create an account
4. **Given** I am an anonymous user, **When** I click the chatbot, **Then** I see a banner: "Sign up for personalized responses based on your programming experience"
5. **Given** I ask multiple questions anonymously, **When** I later sign up and ask the same questions, **Then** I notice improved relevance in personalized responses

---

### User Story 4 - Profile Management and Preference Updates (Priority: P2)

As a **signed-in user**, I can view and update my profile information (email, programming experience), and changes immediately affect my personalized chatbot responses.

**Why this priority**: While important for long-term user satisfaction, this is P2 because users can initially sign up with correct information. Profile editing is a quality-of-life feature that enhances the experience but isn't critical for MVP.

**Independent Test**: Can be tested by signing in, navigating to profile settings, changing programming experience from "Python" to "C++", saving, then asking a coding question and verifying that C++ examples are now prioritized.

**Acceptance Scenarios**:

1. **Given** I am signed in, **When** I click my profile name in the header, **Then** I see a dropdown with "Profile Settings" option
2. **Given** I am on my profile settings page, **When** I view my information, **Then** I see my email and current programming experience
3. **Given** I want to change my programming experience, **When** I update the dropdown and click "Save", **Then** I see a success message and my profile is updated
4. **Given** I updated my programming experience to "C++", **When** I ask the chatbot a code question, **Then** responses immediately prioritize C++ examples
5. **Given** I want to change my email, **When** I enter a new email and save, **Then** I receive a verification email to confirm the change

---

### User Story 5 - Personalized Content Ranking (Priority: P2)

As a **signed-in user with Python experience**, when I ask the chatbot questions, the semantic search results are re-ranked to prioritize Python-related content over C++ content.

**Why this priority**: This is the core personalization mechanism. While P2 (because basic auth must work first), this directly fulfills the constitution's promise of "user experience drives content ranking". Without this, auth provides no tangible benefit.

**Independent Test**: Can be tested by signing in with "Python" experience, asking "How do I create a ROS2 node?", and verifying that the chatbot response prominently features Python (rclpy) code examples before or instead of C++ examples.

**Acceptance Scenarios**:

1. **Given** I have "Python" in my profile, **When** I ask about ROS2 nodes, **Then** vector search results containing Python code receive a 1.3x relevance boost
2. **Given** I have "C++" in my profile, **When** I ask about ROS2 topics, **Then** vector search results containing C++ code receive a 1.3x relevance boost
3. **Given** I have "None" in my profile, **When** I ask technical questions, **Then** search results prioritize conceptual explanations over code-heavy content
4. **Given** I have "Both Python and C++" in my profile, **When** I ask coding questions, **Then** search results are balanced and include both languages equally
5. **Given** I updated my profile from "Python" to "C++", **When** I ask the same question twice (before/after), **Then** I observe different code examples in the responses

---

### User Story 6 - Secure Session Management (Priority: P2)

As a **signed-in user**, my session persists across browser refreshes and different pages, but automatically expires after 7 days of inactivity for security.

**Why this priority**: This is P2 because session persistence improves UX but isn't critical for MVP functionality. However, it's essential for a production-ready auth system that users will trust.

**Independent Test**: Can be tested by signing in, closing the browser, reopening it, navigating to the textbook, and verifying that the user is still authenticated without needing to sign in again.

**Acceptance Scenarios**:

1. **Given** I sign in and close my browser, **When** I reopen the browser and visit the site, **Then** I am still authenticated
2. **Given** I haven't visited the site in 7 days, **When** I return, **Then** I am logged out and see the signin page
3. **Given** I sign in on Device A, **When** I open the site on Device B without signing in, **Then** I am not authenticated (sessions are device-specific)
4. **Given** I sign in with "Remember Me" unchecked, **When** I close the browser, **Then** my session ends and I must sign in again
5. **Given** my session is about to expire (day 6), **When** I interact with the site, **Then** the expiration resets to 7 days from the last activity

---

### Edge Cases

- What happens when a user signs up with an email but their email provider blocks verification emails?
- How does the system handle concurrent signin attempts from different devices using the same account?
- What occurs if a user tries to update their profile while their session has expired?
- How does the backend respond when Neon DB connection fails during signin?
- What happens if a user's programming experience preference is deleted/corrupted in the database?
- How does personalization work for users who signed up before the programming experience field was added (legacy users)?
- What occurs if Qdrant returns no results matching the user's programming experience filter?

## Requirements *(mandatory)*

### Functional Requirements

#### Authentication (Better-Auth Integration)
- **FR-001**: System MUST integrate Better-Auth library for authentication in the FastAPI backend
- **FR-002**: System MUST expose REST API endpoints: `POST /auth/signup`, `POST /auth/signin`, `POST /auth/signout`, `GET /auth/me`
- **FR-003**: System MUST validate email format (RFC 5322) and password strength (min 8 chars, 1 uppercase, 1 lowercase, 1 number)
- **FR-004**: System MUST hash passwords using bcrypt with minimum 10 salt rounds before storing in database
- **FR-005**: System MUST generate secure session tokens (JWT or similar) with 7-day expiration
- **FR-006**: System MUST return appropriate HTTP status codes (200 OK, 401 Unauthorized, 409 Conflict for duplicate email, 422 Validation Error)
- **FR-007**: System MUST implement CORS configuration allowing auth requests from production Vercel domain

#### User Profile & Data Collection
- **FR-008**: System MUST create a `users` table in Neon DB with fields: `id` (UUID), `email` (unique), `password_hash`, `programming_experience`, `created_at`, `updated_at`
- **FR-009**: Signup form MUST include dropdown for programming experience: "None", "Python", "C++", "Both Python and C++"
- **FR-010**: System MUST store programming experience selection in user profile during signup
- **FR-011**: System MUST provide `/auth/me` endpoint returning current user's email and programming experience (excluding password)
- **FR-012**: System MUST provide `PUT /auth/me` endpoint allowing users to update their programming experience

#### Personalization Mechanism
- **FR-013**: Chatbot API (`/chat`, `/stream-chat`) MUST accept optional `Authorization: Bearer <token>` header for authenticated requests
- **FR-014**: System MUST extract user profile from JWT token when present, otherwise treat as anonymous request
- **FR-015**: System MUST apply relevance score boosting to Qdrant search results based on programming experience:
  - "Python" → Boost results containing "python", "rclpy", ".py" by 1.3x
  - "C++" → Boost results containing "c++", "rclcpp", ".cpp", ".hpp" by 1.3x
  - "None" → Boost results containing "explanation", "overview", "concept" by 1.2x
  - "Both" → No language-specific boost (balanced results)
- **FR-016**: System MUST NOT block anonymous users from accessing chatbot (auth is optional)
- **FR-017**: System MUST include personalization indicator in chatbot response metadata (e.g., `"personalized": true/false`)

#### Frontend Integration
- **FR-018**: Frontend MUST display "Sign In / Sign Up" button in header when user is not authenticated
- **FR-019**: Frontend MUST display user's email and "Profile / Sign Out" dropdown in header when authenticated
- **FR-020**: Frontend MUST create modal dialogs for signin and signup forms (not separate pages)
- **FR-021**: Frontend MUST store JWT token in secure httpOnly cookie or localStorage after successful signin
- **FR-022**: Frontend MUST include `Authorization` header in all chatbot API requests when user is authenticated
- **FR-023**: Frontend MUST display profile settings modal allowing users to view/edit programming experience
- **FR-024**: Frontend MUST show informational banner to anonymous users: "Sign up for personalized responses"

### Non-Functional Requirements

#### Security
- **NFR-001**: Password hashing MUST use bcrypt with minimum 10 rounds (OWASP recommendation)
- **NFR-002**: Session tokens MUST expire after 7 days of inactivity
- **NFR-003**: Sensitive endpoints (`/auth/me`, `/auth/signout`) MUST require valid authentication token
- **NFR-004**: System MUST prevent timing attacks on signin by using constant-time password comparison
- **NFR-005**: System MUST sanitize all user inputs to prevent SQL injection and XSS attacks

#### Performance
- **NFR-006**: Signin request MUST complete within 2 seconds (excluding network latency)
- **NFR-007**: Personalized chatbot queries MUST complete within 5 seconds (matching current non-personalized performance)
- **NFR-008**: Database connection pool MUST maintain minimum 5 connections to Neon DB for concurrent user requests

#### Reliability
- **NFR-009**: System MUST gracefully degrade to anonymous mode if Neon DB connection fails (chatbot still works)
- **NFR-010**: System MUST log authentication failures (invalid password, missing token) for security monitoring
- **NFR-011**: System MUST handle expired tokens gracefully by returning 401 Unauthorized (not 500 Internal Server Error)

### Key Entities

- **User**: Individual with account credentials (email, password, programming experience)
- **Session Token (JWT)**: Secure token proving user authentication, stored in cookie/localStorage
- **User Profile**: Database record containing user preferences (programming_experience field)
- **Personalization Context**: User preferences extracted from JWT and passed to Qdrant query
- **Anonymous User**: Visitor without authentication using generic chatbot responses
- **Neon DB**: PostgreSQL database storing user accounts and profiles
- **Better-Auth Library**: Authentication framework integrated into FastAPI backend

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can successfully sign up, sign in, and sign out with 100% success rate (no auth bugs)
- **SC-002**: Signed-in users with "Python" experience receive Python code examples in 80%+ of technical queries
- **SC-003**: Signed-in users with "C++" experience receive C++ code examples in 80%+ of technical queries
- **SC-004**: Anonymous users can access all chatbot functionality without errors or forced signin prompts
- **SC-005**: Profile updates (programming experience changes) reflect in chatbot responses within 1 request (no caching delay)
- **SC-006**: Session persistence works across browser refreshes (95% of users remain authenticated)
- **SC-007**: Zero password leaks in logs, error messages, or API responses (verified by security audit)
- **SC-008**: Average chatbot response time for authenticated users increases by no more than 200ms compared to anonymous users

## Scope *(mandatory)*

### In Scope

- Better-Auth integration into existing FastAPI backend (`rag-backend/chatbot/app.py`)
- Neon DB PostgreSQL database setup with `users` table
- Signup/Signin REST API endpoints (`/auth/*`)
- User profile schema with programming_experience field
- Frontend auth UI (signin/signup modals, profile settings modal)
- JWT session token generation and validation
- Personalization algorithm: relevance score boosting based on programming experience
- Modifying `/chat` and `/stream-chat` endpoints to accept optional Authorization header
- Anonymous user support (auth is optional, not required)
- Frontend header with signin/signup button and profile dropdown
- Secure password hashing with bcrypt

### Out of Scope

- Social authentication (Google, GitHub OAuth) - future enhancement
- Email verification for signups - trust-based initial release
- Password reset functionality - can be added later
- Multi-factor authentication (MFA) - future security enhancement
- User roles or permissions (admin/user) - not needed for personalization
- Collecting additional profile fields (ROS2 experience, hardware access, learning goals) - phase 2
- Subscription tiers or payment integration - no monetization yet
- Advanced personalization (collaborative filtering, ML-based recommendations) - start with rule-based
- Mobile app authentication - web-only for now
- Session management across multiple devices - device-specific sessions only

## Assumptions *(mandatory)*

- Neon DB free tier provides sufficient database capacity for user accounts (up to 10GB storage)
- Better-Auth library supports FastAPI integration (may require custom adapter)
- FastAPI backend can be extended with auth routes without breaking existing chatbot functionality
- Programming experience is the most impactful personalization factor (per user selection)
- Users are honest about their programming experience (no need for skill verification tests)
- Qdrant vector search results include metadata that allows identifying Python vs C++ content
- 7-day session expiration is acceptable to users (balance between convenience and security)
- Frontend can store JWT tokens in localStorage without significant security concerns (no highly sensitive data)
- Current Qdrant index includes sufficient Python and C++ content for effective personalization

## Dependencies *(include if applicable)*

### External Dependencies

- **Better-Auth**: Authentication library for FastAPI (need to verify Python support; may use alternative like FastAPI-Users)
- **Neon DB**: Serverless PostgreSQL database for user account storage
- **SQLAlchemy or Prisma**: ORM for database interactions (SQLAlchemy likely for Python)
- **bcrypt**: Password hashing library (`pip install bcrypt`)
- **PyJWT**: JWT token generation and validation (`pip install pyjwt`)
- **python-multipart**: Required for FastAPI form data parsing (`pip install python-multipart`)

### Internal Dependencies

- Existing FastAPI backend at `rag-backend/chatbot/app.py`
- Existing Qdrant integration in `rag-backend/chatbot/config.py` and query logic
- Frontend React components in `my-website/src/components/`
- Frontend API client at `my-website/src/components/ChatBot/api.ts`
- Existing CORS configuration in FastAPI backend

## Risks *(include if applicable)*

### Technical Risks

- **Risk**: Better-Auth may not have mature Python/FastAPI support (primarily Node.js library)
  - **Mitigation**: Use FastAPI-Users or authlib as alternative; Better-Auth is ideal but not mandatory

- **Risk**: Personalization algorithm may not significantly improve user experience (score boosting insufficient)
  - **Mitigation**: Start with simple rule-based boosting, measure user satisfaction, iterate based on feedback

- **Risk**: Neon DB connection failures may break authentication and prevent signin
  - **Mitigation**: Implement graceful degradation - allow anonymous chatbot access when DB is down

- **Risk**: JWT token storage in localStorage may expose tokens to XSS attacks
  - **Mitigation**: Use httpOnly cookies instead; sanitize all user inputs to prevent XSS

- **Risk**: Adding auth may slow down chatbot responses due to token validation overhead
  - **Mitigation**: Optimize JWT validation with caching; ensure auth adds <200ms latency

### Operational Risks

- **Risk**: Users may forget passwords and have no reset mechanism (out of scope)
  - **Mitigation**: Document this limitation; prioritize password reset in phase 2

- **Risk**: Email harvesting if signup endpoint is publicly accessible without rate limiting
  - **Mitigation**: Implement rate limiting on signup endpoint (e.g., 5 signups per IP per hour)

- **Risk**: Database migration may fail or corrupt existing Qdrant data
  - **Mitigation**: Neon DB and Qdrant are separate; no risk to existing textbook content

## Open Questions

1. **Better-Auth Library Compatibility**: Does Better-Auth officially support Python/FastAPI? If not, should we use FastAPI-Users or authlib instead?
   - **Research needed**: Check Better-Auth documentation for Python bindings

2. **JWT Storage Method**: Should we use localStorage (easier) or httpOnly cookies (more secure) for token storage?
   - **Decision needed**: Evaluate security vs. implementation complexity tradeoff

3. **Personalization Granularity**: Should programming experience be a simple dropdown or multi-select with proficiency levels (Beginner/Intermediate/Advanced)?
   - **Current decision**: Simple dropdown per user selection; proficiency levels are out of scope

4. **Neon DB Schema Migration**: How do we handle schema changes if we need to add fields to the users table later?
   - **Proposed solution**: Use Alembic for database migrations with FastAPI

5. **Anonymous User Tracking**: Should we use cookies to track anonymous users for analytics without forcing signup?
   - **Decision needed**: Clarify if anonymous analytics is in scope for this feature

6. **Session Refresh**: Should we implement automatic token refresh (sliding sessions) or require re-signin after 7 days?
   - **Proposed solution**: Sliding sessions - any activity resets the 7-day expiration

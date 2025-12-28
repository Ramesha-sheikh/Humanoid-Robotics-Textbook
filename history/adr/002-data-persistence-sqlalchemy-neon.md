# ADR-002: Data Persistence with SQLAlchemy and Neon DB

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-27
- **Feature:** better-auth-signup-signin
- **Context:** The authentication system requires persistent storage for user accounts (email, password hash, programming experience). The application uses Python/FastAPI backend and needs an ORM for database interactions. Database must support connection pooling (min 5 connections per NFR-008), provide migration tooling, and integrate seamlessly with FastAPI. Neon DB offers serverless PostgreSQL aligned with existing Hugging Face Spaces deployment.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? YES - foundational data layer for user accounts
     2) Alternatives: Multiple viable options considered with tradeoffs? YES - Prisma, raw SQL, other databases
     3) Scope: Cross-cutting concern (not an isolated detail)? YES - affects migrations, queries, testing, scaling
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Implement data persistence using:

- **Database**: Neon DB (Serverless PostgreSQL 15)
- **ORM**: SQLAlchemy 2.0+
- **Migration Tool**: Alembic (SQLAlchemy companion)
- **Connection Management**: SQLAlchemy connection pooling (5 min connections, 10 max overflow)
- **Schema Definition**: Declarative SQLAlchemy models with Pydantic validation
- **Database URL**: Stored in environment variable `NEON_DATABASE_URL` (not hardcoded)

User schema:
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

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

- **Python Ecosystem Standard**: SQLAlchemy is de facto ORM for Python web applications (15+ years production use)
- **FastAPI Native**: FastAPI documentation uses SQLAlchemy in all official examples (ecosystem fit)
- **Connection Pooling Built-In**: Automatic pooling satisfies NFR-008 (minimum 5 connections) without additional configuration
- **Type Safety**: Pydantic integration provides type hints comparable to Prisma's TypeScript safety
- **Migration Support**: Alembic handles schema evolution with auto-generated migrations (`alembic revision --autogenerate`)
- **Serverless-Compatible**: Neon DB provides serverless PostgreSQL (auto-scaling, pay-per-use, 99.9% SLA)
- **Free Tier Available**: Neon DB free tier (10GB storage, 100 compute hours) sufficient for MVP (est. 100k users)
- **Zero-Downtime Migrations**: Neon DB supports online schema changes without downtime
- **Mature Ecosystem**: Extensive documentation, community support, proven production reliability

<!-- Example: Integrated tooling, excellent DX, fast deploys, strong TypeScript support -->

### Negative

- **Verbosity**: SQLAlchemy queries more verbose than Prisma's type-safe API (e.g., `session.query(User).filter(User.email == email)` vs `prisma.user.findUnique({where: {email}})`)
- **Learning Curve**: Complex queries require understanding SQLAlchemy's ORM patterns (Core vs ORM, lazy loading)
- **Abstraction Overhead**: ORM adds latency vs raw SQL (~10-20ms per query for ORM layer)
- **Not as Modern**: Prisma's auto-generated client and migration preview feel more "modern" than SQLAlchemy
- **Database Vendor Lock-In**: Neon DB-specific features (branching, instant rollback) create mild vendor dependency
- **Cold Start Latency**: Neon DB serverless has ~100-200ms cold start if idle (acceptable for auth, not real-time apps)

<!-- Example: Vendor lock-in to Vercel, framework coupling, learning curve -->

## Alternatives Considered

### Alternative 1: Prisma (TypeScript-First ORM)
- **Components**: Prisma Client, Prisma Schema, Prisma Migrate
- **Why Rejected**: Designed for Node.js/TypeScript; Python support (Prisma Client Python) is experimental and limited. Not mature enough for production. Would require Node.js backend or transpilation complexity. Doesn't integrate with FastAPI ecosystem.

### Alternative 2: Raw SQL with psycopg2 (PostgreSQL Adapter)
- **Components**: psycopg2 driver, manual query construction, hand-written migrations
- **Why Rejected**: Manual query construction is error-prone (SQL injection risk). Verbose code for CRUD operations. No automatic migrations. Requires parameterization discipline. Higher maintenance burden. Loses type safety (no ORM models).

### Alternative 3: Supabase (Firebase Alternative with PostgreSQL)
- **Components**: Supabase hosted PostgreSQL, auto-generated REST API, client libraries
- **Why Rejected**: Adds unnecessary abstraction layer (auto-generated API we don't need). Vendor lock-in to Supabase ecosystem. More expensive than Neon DB at scale. Overkill for simple user account storage.

### Alternative 4: MongoDB with Motor (NoSQL)
- **Components**: MongoDB database, Motor async driver, document-based schema
- **Why Rejected**: Overkill for simple relational data (users table). PostgreSQL provides better data integrity (UNIQUE constraints, foreign keys). Team familiarity with SQL. No compelling reason to use NoSQL for structured user accounts.

### Alternative 5: SQLite (Embedded Database)
- **Components**: SQLite file-based database, SQLAlchemy with SQLite dialect
- **Why Rejected**: Not suitable for serverless deployment (HF Spaces ephemeral filesystem). No concurrent writes (single file lock). Not scalable for multi-instance backend. Acceptable for local development only.

<!-- Group alternatives by cluster:
     Alternative Stack A: Remix + styled-components + Cloudflare
     Alternative Stack B: Vite + vanilla CSS + AWS Amplify
     Why rejected: Less integrated, more setup complexity
-->

## References

- Feature Spec: [specs/002-better-auth-signup-signin/spec.md](../../specs/002-better-auth-signup-signin/spec.md)
- Implementation Plan: [specs/002-better-auth-signup-signin/plan.md](../../specs/002-better-auth-signup-signin/plan.md#decision-4-sqlalchemy-orm-with-neon-db-not-prisma-or-raw-sql)
- Data Model: [specs/002-better-auth-signup-signin/data-model.md](../../specs/002-better-auth-signup-signin/data-model.md)
- Related ADRs:
  - ADR-001 (JWT auth system stores user data in this database)
- Evaluator Evidence: [history/prompts/better-auth-signup-signin/0001-implementation-plan-better-auth-signup-signin.plan.prompt.md](../prompts/better-auth-signup-signin/0001-implementation-plan-better-auth-signup-signin.plan.prompt.md)

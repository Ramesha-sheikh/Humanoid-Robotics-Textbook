# ADR-005: Environment Variables for Secrets Management

- **Status:** Accepted
- **Date:** 2025-12-10
- **Feature:** rag-chatbot-integration
- **Context:** System requires 5 API keys/secrets: Gemini API key, Qdrant URL + API key, Neon Postgres connection string, and CORS origins. Must be secure (no hardcoding), deployable to Railway (backend) + Vercel (frontend), and work locally for development. Hackathon timeline requires simple, standard approach.

## Decision

Use **`.env` files for local development** and **platform-native environment variable injection for production** (Railway + Vercel).

**Architecture:**

1. **Local Development**: `.env` file (gitignored)
   ```bash
   # backend/.env
   GEMINI_API_KEY=AIzaSy...
   QDRANT_URL=https://xyz.qdrant.io
   QDRANT_API_KEY=pQrT...
   NEON_DATABASE_URL=postgresql://user:pass@ep-xyz.neon.tech/db
   CORS_ORIGINS=http://localhost:3000,http://localhost:3001

   # frontend/.env.local
   NEXT_PUBLIC_API_URL=http://localhost:8000
   ```

2. **Production Deployment**:
   - **Railway (backend)**: Set environment variables in Railway dashboard (auto-injected on build)
   - **Vercel (frontend)**: Set environment variables in Vercel project settings (auto-injected on build)

3. **No Secret Management Service**: No AWS Secrets Manager, HashiCorp Vault, or Doppler (overkill for hackathon)

4. **Loading Strategy**:
   ```python
   # backend/app/config.py
   from pydantic_settings import BaseSettings

   class Settings(BaseSettings):
       gemini_api_key: str
       qdrant_url: str
       qdrant_api_key: str
       neon_database_url: str
       cors_origins: list[str] = ["http://localhost:3000"]

       class Config:
           env_file = ".env"
           env_file_encoding = "utf-8"

   settings = Settings()  # Automatically loads from .env or environment
   ```

5. **Security Practices**:
   - `.env` in `.gitignore` (never commit)
   - `.env.example` with dummy values for documentation
   - Validate all secrets on startup (fail fast if missing)
   - No logging of secret values

## Consequences

### Positive

- **Industry standard**: `.env` pattern is universally understood and supported
- **Platform-native**: Railway and Vercel automatically inject environment variables (zero config)
- **Fast setup**: Team members create `.env` from `.env.example` in 30 seconds
- **No external dependencies**: No secret management service to configure or pay for
- **Type-safe**: Pydantic validates environment variables at startup
- **Easy debugging**: Can override variables locally without changing code
- **Deployment simplicity**: Railway/Vercel auto-detect `.env` schema from code
- **No vendor lock-in**: Standard approach works on any platform (AWS, GCP, Fly.io, etc.)

### Negative

- **No rotation automation**: Must manually update secrets in Railway/Vercel dashboards
- **No audit trail**: Cannot track who accessed or changed secrets
- **Limited access control**: Anyone with Railway/Vercel access sees all secrets
- **No versioning**: Cannot rollback to previous secret values
- **Manual sync required**: Must update secrets in 2 places (Railway + Vercel)
- **Risk of exposure**: If `.env` accidentally committed, secrets leak (mitigated: `.gitignore` + pre-commit hooks)

## Alternatives Considered

### Alternative A: Hardcode secrets in code
- **Approach**: Store API keys directly in Python files
- **Why rejected**:
  - **Security disaster**: Secrets exposed in Git history, public repos, and logs
  - **Cannot deploy**: Different keys for dev/staging/prod require code changes
  - **Violates best practices**: Every security audit fails immediately

### Alternative B: AWS Secrets Manager or GCP Secret Manager
- **Approach**: Store secrets in cloud service, fetch at runtime
- **Why rejected**:
  - **Overkill for hackathon**: Takes 2-3 hours to configure IAM roles, SDK setup, and error handling
  - **Adds complexity**: Need AWS/GCP account, separate deployment step, and SDK dependencies
  - **Cost**: AWS Secrets Manager charges $0.40/secret/month (small but unnecessary)
  - **Latency penalty**: Fetching secrets at runtime adds 50-100ms startup time
  - **Not Railway/Vercel-native**: Requires custom init scripts

### Alternative C: HashiCorp Vault or Doppler
- **Approach**: Use dedicated secret management platform
- **Why rejected**:
  - **Enterprise-grade overkill**: Designed for teams with 100+ secrets and compliance requirements
  - **Learning curve**: Takes 1-2 days to learn Vault policies or Doppler CLI
  - **Deployment complexity**: Must host Vault or integrate Doppler into CI/CD
  - **Cost**: Doppler starts at $7/user/month, Vault self-hosted requires infrastructure
  - **Not needed for 5 secrets**: `.env` is sufficient for small projects

### Alternative D: Encrypted .env files (e.g., `sops`, `git-crypt`)
- **Approach**: Commit encrypted `.env.encrypted`, decrypt on deployment
- **Why rejected**:
  - **Adds tooling complexity**: Team must install `sops` or `git-crypt` and manage decryption keys
  - **Key distribution problem**: Still need to securely share decryption key (shifts problem)
  - **Railway/Vercel incompatible**: Platforms expect plain `.env` or dashboard variables
  - **Overhead for hackathon**: Takes 1-2 hours to setup encryption pipeline

### Alternative E: Config server (Spring Cloud Config, Consul)
- **Approach**: Centralized config server that applications query at startup
- **Why rejected**:
  - **Massive over-engineering**: Designed for microservices with 50+ services
  - **Infrastructure overhead**: Must deploy and maintain separate config server
  - **Single point of failure**: If config server down, entire application fails
  - **Not Railway/Vercel-native**: Requires custom networking and service discovery

## References

- Feature Spec: `specs/rag-chatbot-integration/spec.md` (TC-001 through TC-003, DEP-001 through DEP-003)
- Implementation Plan: `specs/rag-chatbot-integration/plan.md` (pending)
- Related ADRs:
  - ADR-001 (Gemini API key stored in GEMINI_API_KEY)
  - ADR-004 (Qdrant credentials in QDRANT_URL and QDRANT_API_KEY)
- Railway Docs: https://docs.railway.app/guides/variables
- Vercel Docs: https://vercel.com/docs/projects/environment-variables
- Pydantic Settings: https://docs.pydantic.dev/latest/concepts/pydantic_settings/

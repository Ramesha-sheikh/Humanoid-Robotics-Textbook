# Research: Deploy Backend to Hugging Face Spaces

**Date**: 2025-12-25
**Feature**: 001-deploy-backend-hf
**Phase**: 0 (Research & Discovery)

## Overview

This document captures research findings for deploying a FastAPI backend to Hugging Face Spaces, including Docker configuration, environment variable management, CORS setup, and production deployment best practices.

## Research Areas

### 1. Hugging Face Spaces Deployment

**Decision**: Use Docker SDK on Hugging Face Spaces

**Rationale**:
- Hugging Face Spaces supports multiple SDKs (Streamlit, Gradio, Docker)
- Docker SDK provides full control over the deployment environment
- FastAPI applications are best deployed in containers with explicit dependency management
- Port 7860 is the standard HTTP port for HF Spaces Docker deployments

**Alternatives Considered**:
- **Gradio SDK**: Not suitable - designed for ML demos, not REST APIs
- **Streamlit SDK**: Not suitable - designed for data apps, not backend APIs
- **Direct Python deployment**: HF Spaces doesn't support this for FastAPI

**Best Practices**:
- Use official Python slim images (python:3.11-slim) for smaller container size
- Expose port 7860 (HF Spaces requirement)
- Use uvicorn as ASGI server with --host 0.0.0.0
- Include health check endpoint for monitoring
- Set appropriate resource limits in Dockerfile

### 2. Docker Configuration for FastAPI

**Decision**: Use multi-stage build with Python 3.11-slim base image

**Rationale**:
- Python 3.11-slim balances size (~45MB) with functionality
- Single-stage build sufficient for this simple deployment
- Uvicorn with standard workers handles FastAPI efficiently
- No need for complex build steps (no compilation, no frontend assets)

**Dockerfile Structure**:
```dockerfile
FROM python:3.11-slim
WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY . .
EXPOSE 7860
CMD ["uvicorn", "app:app", "--host", "0.0.0.0", "--port", "7860"]
```

**Alternatives Considered**:
- **Alpine Linux**: Smaller but has compatibility issues with some Python packages
- **Multi-stage build**: Overkill for this deployment (no compilation needed)
- **Python 3.12**: Stick with 3.11 for stability and compatibility

### 3. Environment Variables and Secrets Management

**Decision**: Use Hugging Face Spaces Secrets feature for all API keys

**Rationale**:
- HF Spaces provides built-in secrets management (similar to GitHub Secrets)
- Environment variables are injected at runtime, never exposed in code
- Secrets are encrypted at rest and in transit
- Easy to rotate keys without code changes
- Free tier includes secrets management

**Required Environment Variables**:
- `COHERE_API_KEY`: Cohere API key for embeddings and chat
- `QDRANT_URL`: Qdrant cloud instance URL
- `QDRANT_API_KEY`: Qdrant API key
- `COLLECTION_NAME`: Vector collection name (default: "book")

**Best Practices**:
- Create `.env.example` with placeholder values for documentation
- Add `.env` to `.gitignore` (verify it's there)
- Use `python-dotenv` for local development
- Read env vars using `os.getenv()` with no defaults in production code
- Validate all required env vars on startup (fail fast if missing)

### 4. CORS Configuration for Production

**Decision**: Configure CORS middleware with explicit Vercel domain allowlist

**Rationale**:
- Vercel frontend makes cross-origin requests to backend
- CORS must be configured on backend (can't be fixed on frontend)
- Explicit allowlist more secure than wildcard (*)
- Need to handle preflight OPTIONS requests

**CORS Configuration**:
```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://humanoid-robotics-textbook-psi.vercel.app",  # Production
        "http://localhost:3000"  # Local development
    ],
    allow_credentials=True,
    allow_methods=["*"],  # Allow all HTTP methods
    allow_headers=["*"],  # Allow all headers
)
```

**Alternatives Considered**:
- **Wildcard (*)**: Less secure, allows any domain to make requests
- **Regex patterns**: Overkill for single production domain
- **Custom middleware**: FastAPI's built-in middleware is sufficient

**Common CORS Issues**:
- Missing preflight OPTIONS handler → FastAPI handles automatically
- Wrong domain in allowlist → Check exact production URL (https://, no trailing slash)
- Credentials not allowed → Set `allow_credentials=True`

### 5. Deployment Workflow

**Decision**: Manual deployment via HF Spaces web interface or Git push

**Rationale**:
- HF Spaces supports both web upload and Git integration
- Git push workflow better for version control
- Automatic rebuilds on git push (similar to Vercel)
- No need for complex CI/CD for simple backend deployment

**Deployment Steps**:
1. Create new Space on huggingface.co (SDK: Docker)
2. Connect Git repository OR upload files manually
3. Configure secrets in Space settings
4. Push code → automatic build and deploy
5. Verify health endpoint responds
6. Update frontend with production URL
7. Test all endpoints from production frontend

**Alternatives Considered**:
- **GitHub Actions CI/CD**: Overkill for manual deployment
- **Docker Hub + pull**: Extra step not needed with HF's built-in container registry

### 6. Health Check and Monitoring

**Decision**: Use existing `/health` endpoint with service status checks

**Rationale**:
- Health endpoint already exists in backend code
- Returns connection status for Cohere and Qdrant
- HF Spaces automatically monitors HTTP endpoints
- Simple curl-based monitoring sufficient for current scale

**Health Check Response**:
```json
{
  "status": "ok",
  "version": "1.0",
  "services": {
    "cohere": "connected",
    "qdrant": "connected"
  }
}
```

**Monitoring Strategy**:
- HF Spaces dashboard shows uptime and logs
- Manual curl checks during deployment
- Frontend error handling provides user-facing monitoring
- Can add Uptime Robot or similar tool later if needed

### 7. Frontend URL Updates

**Decision**: Replace localhost:8001 with HF Spaces production URL in frontend

**Rationale**:
- Frontend currently hardcoded to call http://localhost:8001
- Need to update to https://<space-name>.hf.space (or custom HF domain)
- Update in 2 files: DocItem/Content/index.js and potentially ChatBot component

**Files to Update**:
- `my-website/src/theme/DocItem/Content/index.js` (line 69: fetch URL)
- Verify ChatBot component doesn't have hardcoded localhost

**Best Practice**:
- Use environment variable (REACT_APP_BACKEND_URL) for configurability
- Or use conditional: `process.env.NODE_ENV === 'production' ? 'https://...' : 'http://localhost:8001'`
- Test thoroughly in production before switching all users

## Open Questions Resolved

1. **Q**: Which Python version for Docker image?
   **A**: Python 3.11-slim (stable, compatible with all dependencies)

2. **Q**: How to handle CORS for multiple domains?
   **A**: Explicit allowlist with production Vercel domain and localhost for dev

3. **Q**: Where to store API keys?
   **A**: HF Spaces Secrets (environment variables), never in code

4. **Q**: Which port to use?
   **A**: Port 7860 (HF Spaces standard for Docker SDK)

5. **Q**: How to monitor backend health?
   **A**: Existing /health endpoint + HF Spaces dashboard logs

## References

- Hugging Face Spaces Docker documentation: https://huggingface.co/docs/hub/spaces-sdks-docker
- FastAPI deployment best practices: https://fastapi.tiangolo.com/deployment/
- CORS configuration guide: https://fastapi.tiangolo.com/tutorial/cors/
- Uvicorn production deployment: https://www.uvicorn.org/deployment/

## Next Steps

Proceed to Phase 1: Create deployment configuration files (Dockerfile, requirements.txt, README.md) and API contracts documentation.

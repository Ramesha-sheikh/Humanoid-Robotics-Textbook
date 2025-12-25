# Rollback Reference

## Original Configuration (Before HF Spaces Deployment)

**Backend URL**: `http://localhost:8001`

**Frontend Configuration**:
- Translation component: `my-website/src/theme/DocItem/Content/index.js` line 69
  - Original: `fetch('http://localhost:8001/translate', ...)`

**CORS Configuration** (`rag-backend/chatbot/app.py` lines 14-20):
```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

## Rollback Steps

If deployment fails and you need to revert:

1. **Restore app.py configuration**:
   ```bash
   cp rag-backend/chatbot/app.py.backup rag-backend/chatbot/app.py
   ```

2. **Revert frontend URL** (if changed):
   - Edit `my-website/src/theme/DocItem/Content/index.js` line 69
   - Change back to: `http://localhost:8001/translate`

3. **Start local backend**:
   ```bash
   cd rag-backend/chatbot
   python -m uvicorn app:app --host 127.0.0.1 --port 8001 --reload
   ```

4. **Verify local setup working**:
   - Test health: `curl http://localhost:8001/health`
   - Test translation from local Docusaurus (port 3000)

## Deployment Date

**Date**: 2025-12-25
**Original backup**: `rag-backend/chatbot/app.py.backup`

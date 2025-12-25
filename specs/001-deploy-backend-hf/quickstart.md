# Quickstart: Deploy Backend to Hugging Face Spaces

**Date**: 2025-12-25
**Feature**: 001-deploy-backend-hf
**Estimated Time**: 30-45 minutes

## Prerequisites

Before starting deployment:

- [ ] Hugging Face account created (free tier)
- [ ] Cohere API key available
- [ ] Qdrant URL and API key available
- [ ] Access to this repository
- [ ] Vercel production URL known

## Step 1: Prepare Deployment Files

### 1.1 Create requirements.txt

Navigate to `rag-backend/chatbot/` and create `requirements.txt`:

```txt
fastapi==0.104.1
uvicorn[standard]==0.24.0
cohere==4.37
qdrant-client==1.7.0
pydantic==2.5.0
python-dotenv==1.0.0
pyyaml==6.0.1
```

### 1.2 Create Dockerfile

Create `rag-backend/chatbot/Dockerfile`:

```dockerfile
FROM python:3.11-slim

WORKDIR /app

# Copy and install dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY . .

# Expose HF Spaces standard port
EXPOSE 7860

# Start FastAPI with uvicorn
CMD ["uvicorn", "app:app", "--host", "0.0.0.0", "--port", "7860"]
```

### 1.3 Create .env.example

Create `rag-backend/chatbot/.env.example`:

```env
# Cohere API Configuration
COHERE_API_KEY=your_cohere_api_key_here

# Qdrant Configuration
QDRANT_URL=https://your-qdrant-instance.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here
COLLECTION_NAME=book

# Application Configuration
SITEMAP_URL=https://humanoid-robotics-textbook-psi.vercel.app/sitemap.xml
```

### 1.4 Verify .gitignore

Check that `.gitignore` includes:

```
.env
*.env
!.env.example
```

### 1.5 Update CORS Configuration

Edit `rag-backend/chatbot/app.py` line 14-20 to include production domain:

```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://humanoid-robotics-textbook-psi.vercel.app",  # Production
        "http://localhost:3000",  # Local dev
        "http://localhost:3001"   # Alternative local port
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

## Step 2: Create Hugging Face Space

### 2.1 Create New Space

1. Go to [huggingface.co/spaces](https://huggingface.co/new-space)
2. Click "Create new Space"
3. Configure:
   - **Owner**: Your HF username
   - **Space name**: `humanoid-robotics-chatbot` (or your preferred name)
   - **License**: Apache 2.0 or MIT
   - **SDK**: **Docker** (important!)
   - **Visibility**: Public (for free tier)

4. Click "Create Space"

### 2.2 Note Your Space URL

Your Space URL will be:
```
https://[your-username]-humanoid-robotics-chatbot.hf.space
```

Save this URL - you'll need it for frontend configuration.

## Step 3: Upload Code to Space

### Option A: Git Push (Recommended)

1. Clone your new Space:
   ```bash
   git clone https://huggingface.co/spaces/[your-username]/humanoid-robotics-chatbot
   cd humanoid-robotics-chatbot
   ```

2. Copy backend files:
   ```bash
   cp -r /path/to/rag-backend/chatbot/* .
   ```

3. Commit and push:
   ```bash
   git add .
   git commit -m "Initial backend deployment"
   git push
   ```

### Option B: Web Upload

1. Go to your Space page
2. Click "Files" tab
3. Click "Add file" → "Upload files"
4. Upload all files from `rag-backend/chatbot/`:
   - `app.py`
   - `config.py`
   - `requirements.txt`
   - `Dockerfile`
   - `.env.example` (documentation only)

5. Commit changes

## Step 4: Configure Environment Variables

### 4.1 Add Secrets

1. Go to Space Settings (gear icon)
2. Click "Repository secrets"
3. Add each secret:

   **COHERE_API_KEY**:
   ```
   [your-actual-cohere-key]
   ```

   **QDRANT_URL**:
   ```
   https://[your-instance].qdrant.io
   ```

   **QDRANT_API_KEY**:
   ```
   [your-actual-qdrant-key]
   ```

   **COLLECTION_NAME**:
   ```
   book
   ```

4. Click "Save" after each secret

### 4.2 Restart Space

After adding secrets:
1. Go to Space page
2. Click "⋮" menu → "Factory reboot"
3. Wait for rebuild (2-3 minutes)

## Step 5: Verify Deployment

### 5.1 Check Build Logs

1. Go to Space page
2. Click "Logs" tab
3. Verify no errors:
   - ✅ `pip install` completes
   - ✅ `uvicorn` starts on port 7860
   - ✅ No connection errors

### 5.2 Test Health Endpoint

Open in browser or curl:
```bash
curl https://[your-username]-humanoid-robotics-chatbot.hf.space/health
```

Expected response:
```json
{
  "status": "ok",
  "version": "1.0"
}
```

### 5.3 Test Chat Endpoint

```bash
curl -X POST https://[your-username]-humanoid-robotics-chatbot.hf.space/chat \
  -H "Content-Type: application/json" \
  -d '{"question": "What is a robot?"}'
```

Expected: JSON response with `answer` and `sources`

### 5.4 Test Translation Endpoint

```bash
curl -X POST https://[your-username]-humanoid-robotics-chatbot.hf.space/translate \
  -H "Content-Type: application/json" \
  -d '{
    "page_url": "https://humanoid-robotics-textbook-psi.vercel.app/docs/introduction/",
    "target_language": "urdu"
  }'
```

Expected: JSON with `success: true` and Urdu translation

### 5.5 Test CORS

From browser console on your Vercel site:
```javascript
fetch('https://[your-space-url]/health')
  .then(r => r.json())
  .then(console.log)
```

Expected: No CORS errors, response logged

## Step 6: Update Frontend

### 6.1 Update Translation Component

Edit `my-website/src/theme/DocItem/Content/index.js` line 69:

```javascript
// OLD
const response = await fetch('http://localhost:8001/translate', {

// NEW
const response = await fetch('https://[your-username]-humanoid-robotics-chatbot.hf.space/translate', {
```

### 6.2 Update Chatbot Component (if applicable)

Check `my-website/src/components/RagChatbot/` for any hardcoded localhost URLs and update to production HF Space URL.

### 6.3 Test Locally

```bash
cd my-website
npm start
```

Navigate to a page, click "اردو" button, verify translation works with production backend.

### 6.4 Deploy Frontend

```bash
git add .
git commit -m "Update backend URL to production HF Space"
git push
```

Vercel will auto-deploy. Wait 2-3 minutes.

## Step 7: Production Verification

### 7.1 End-to-End Test

1. Visit production site: https://humanoid-robotics-textbook-psi.vercel.app
2. Open a chapter page
3. Click "اردو" button → verify Urdu translation appears
4. Click chatbot icon → ask a question → verify response

### 7.2 Performance Check

Using browser DevTools Network tab:
- Translation: < 10 seconds ✅
- Chatbot: < 5 seconds ✅
- No CORS errors ✅

### 7.3 Monitor for 24 Hours

Check HF Space logs periodically:
- No repeated errors
- Uptime maintained
- Response times acceptable

## Troubleshooting

### Build Fails

**Symptom**: "ERROR: Could not find a version that satisfies the requirement..."

**Solution**:
- Check `requirements.txt` syntax
- Verify all versions exist on PyPI
- Try removing version pins (e.g., `fastapi` instead of `fastapi==0.104.1`)

### CORS Errors

**Symptom**: "Access to fetch at... has been blocked by CORS policy"

**Solution**:
- Verify `allow_origins` list in `app.py` includes exact Vercel URL
- Check for https:// (not http://)
- No trailing slash in URL
- Factory reboot Space after code changes

### Connection Errors

**Symptom**: "Connection to Cohere/Qdrant failed"

**Solution**:
- Verify all environment variables set in Space settings
- Check API keys are valid (not expired)
- Factory reboot after adding secrets

### 500 Internal Server Error

**Symptom**: Backend returns 500 on all requests

**Solution**:
- Check Space logs for Python errors
- Verify `config.py` reads env vars correctly
- Test health endpoint first to isolate issue

### Translation Returns "No content found"

**Symptom**: Translation endpoint returns `success: false`

**Solution**:
- Verify URL normalization logic in `app.py`
- Check Qdrant collection has data (280 points)
- Test with exact production URL format

## Rollback Plan

If deployment fails:

1. **Revert CORS changes**: Remove HF URL from `allow_origins`, keep localhost only
2. **Revert frontend**: Update URLs back to `localhost:8001`
3. **Keep Space running**: Can fix issues without affecting users
4. **Debug locally**: Test all fixes with local backend first

## Success Criteria Checklist

- [ ] HF Space deployed and running (green status)
- [ ] Health endpoint returns 200 OK
- [ ] Chat endpoint returns valid answers
- [ ] Translation endpoint translates to Urdu
- [ ] CORS allows Vercel domain requests
- [ ] No API keys visible in code/logs
- [ ] Frontend successfully calls production backend
- [ ] End-to-end test passes on production site

## Next Steps

After successful deployment:

1. **Monitor**: Check HF Space logs daily for first week
2. **Document**: Save Space URL in project README
3. **Backup**: Keep copy of working configuration files
4. **Optimize**: Consider upgrading to paid tier if free tier limits reached
5. **Enhance**: Add monitoring tools (Uptime Robot, etc.) if desired

## Support Resources

- HF Spaces Documentation: https://huggingface.co/docs/hub/spaces
- FastAPI Deployment: https://fastapi.tiangolo.com/deployment/
- Project Skills: `.claude/skills/backend-deployment/SKILL.md`
- Project Constitution: `.specify/memory/constitution.md`

---

**Deployment completed successfully?** Proceed to close this feature branch and document lessons learned!

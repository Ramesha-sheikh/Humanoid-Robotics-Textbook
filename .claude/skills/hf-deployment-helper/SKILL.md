---
name: hf-deployment-helper
version: 1.0
author: Humanoid Robotics Textbook Team
description: Generates Hugging Face Space configuration for FastAPI apps
output_format: "Complete deployment package with all config files"
---

# Hugging Face Deployment Helper

Automatically generates all configuration files needed to deploy a FastAPI application to Hugging Face Spaces.

## What This Skill Generates

1. **requirements.txt** - All Python dependencies with versions
2. **Dockerfile** - Optimized for HF Spaces (port 7860)
3. **README.md** - API documentation with endpoints
4. **.env.example** - Template for environment variables
5. **app_config.py** - Configuration management module

## When to Use

Use this skill when:
- Deploying FastAPI backend to Hugging Face Spaces
- Need to expose APIs to Vercel-deployed frontend
- Setting up RAG chatbot or translation service
- Migrating from localhost to production

## Generated File Structure

```
rag-backend/chatbot/
├── app.py                 (existing)
├── config.py              (existing)
├── requirements.txt       (generated)
├── Dockerfile             (generated)
├── README.md              (generated)
└── .env.example           (generated)
```

## Requirements Template

```txt
fastapi==0.104.1
uvicorn[standard]==0.24.0
cohere==4.37
qdrant-client==1.7.0
pydantic==2.5.0
python-dotenv==1.0.0
pyyaml==6.0.1
```

## Dockerfile Template

```dockerfile
FROM python:3.11-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

EXPOSE 7860

CMD ["uvicorn", "app:app", "--host", "0.0.0.0", "--port", "7860"]
```

## Environment Variables

Required secrets in HF Space:
- `COHERE_API_KEY` - Cohere API key for embeddings and chat
- `QDRANT_URL` - Qdrant cloud instance URL
- `QDRANT_API_KEY` - Qdrant API key
- `COLLECTION_NAME` - Vector collection name (default: "book")

## CORS Configuration

Update `app.py` to include production URL:

```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://humanoid-robotics-textbook-psi.vercel.app",
        "http://localhost:3000"
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

## Deployment Checklist

- [ ] All files generated in backend directory
- [ ] Requirements include all dependencies
- [ ] Dockerfile uses port 7860
- [ ] CORS configured for production domain
- [ ] Environment variables documented
- [ ] API endpoints tested locally
- [ ] README includes usage examples

## Validation

After deployment, test:
1. Health endpoint: `GET /health`
2. Translation: `POST /translate`
3. Chat: `POST /chat`
4. Stream chat: `POST /stream-chat`
5. CORS headers present in responses

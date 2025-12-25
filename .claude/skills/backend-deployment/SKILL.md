---
name: backend-deployment
version: 1.0
author: Humanoid Robotics Textbook Team
description: Deploys FastAPI backends to Hugging Face Spaces with configuration
output_format: "Deployment files and step-by-step instructions"
---

# Backend Deployment Skill

Deploy FastAPI backends (translation + RAG chatbot) to Hugging Face Spaces for production access.

## What This Skill Does

1. Creates all necessary deployment files (requirements.txt, Dockerfile, README)
2. Configures CORS for Vercel frontend access
3. Sets up environment variables securely
4. Provides deployment instructions
5. Creates health check and monitoring setup

## Usage

Call this skill when you need to:
- Deploy FastAPI backend to Hugging Face Spaces
- Configure production CORS settings
- Set up environment variables for Cohere/Qdrant
- Create Dockerfile for containerized deployment

## Output

This skill generates:
- `requirements.txt` - Python dependencies
- `Dockerfile` - Container configuration for HF Spaces
- `README.md` - API documentation
- `.env.example` - Environment variable template
- Deployment checklist and instructions

## Configuration Template

### CORS Settings
```python
allow_origins=[
    "https://your-vercel-app.vercel.app",
    "http://localhost:3000"
]
```

### Required Environment Variables
- COHERE_API_KEY
- QDRANT_URL
- QDRANT_API_KEY
- COLLECTION_NAME

## Deployment Steps

1. Create Hugging Face Space (SDK: Docker)
2. Upload deployment files
3. Configure secrets in Space settings
4. Deploy and verify endpoints
5. Update frontend with production URL

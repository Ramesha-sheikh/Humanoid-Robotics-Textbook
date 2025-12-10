# RAG Chatbot Backend

Backend for Humanoid Robotics Textbook RAG Chatbot using:
- **LLM**: Gemini 2.0 Flash
- **Vector DB**: Qdrant Cloud
- **Database**: Neon Postgres
- **Framework**: FastAPI + Python 3.11

## 🚀 Quick Start

### 1. Install Dependencies

```bash
cd backend
python -m venv venv

# Windows
venv\Scripts\activate

# Linux/Mac
source venv/bin/activate

pip install -r requirements.txt
```

### 2. Setup Environment Variables

Create `.env` file:

```bash
cp .env.example .env
```

Edit `.env` and add your API keys:

```env
GEMINI_API_KEY=your_gemini_api_key_here
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here
NEON_DATABASE_URL=postgresql://user:pass@ep-xyz.neon.tech/db
CORS_ORIGINS=http://localhost:3000
```

**Get API Keys:**
- **Gemini**: https://ai.google.dev/ (Google AI Studio)
- **Qdrant**: https://qdrant.tech/ (Free tier)
- **Neon**: https://neon.tech/ (Free tier)

### 3. Ingest Textbook

```bash
python scripts/ingestion.py --pdf path/to/textbook.pdf
```

This will:
1. Extract text from PDF
2. Chunk into 512-token segments
3. Generate embeddings with text-embedding-004
4. Upload to Qdrant Cloud

### 4. Run Backend Server

```bash
cd app
python main.py
```

Server will start at: http://localhost:8000

**API Docs**: http://localhost:8000/docs

## 📡 API Endpoints

### Health Check
```bash
GET /health
```

### Chat (Non-Streaming)
```bash
POST /chat
Content-Type: application/json

{
  "query": "What is inverse kinematics?",
  "mode": "normal"
}
```

### Chat (Streaming)
```bash
POST /chat/stream
Content-Type: application/json

{
  "query": "Explain bipedal locomotion",
  "mode": "normal"
}
```

### Highlight Mode
```bash
POST /chat/stream
Content-Type: application/json

{
  "query": "What sensors are mentioned?",
  "mode": "highlight",
  "highlight_context": {
    "text": "Selected text from textbook...",
    "startPage": 42,
    "endPage": 45,
    "chapterSlug": "kinematics"
  }
}
```

## 🏗️ Project Structure

```
backend/
├── app/
│   ├── main.py           # FastAPI application
│   ├── config.py         # Environment config
│   └── rag.py            # RAG pipeline (Gemini + Qdrant)
├── scripts/
│   └── ingestion.py      # PDF → Qdrant ingestion
├── requirements.txt      # Python dependencies
├── .env.example          # Environment template
└── README.md             # This file
```

## 🧪 Testing

### Test Health Endpoint
```bash
curl http://localhost:8000/health
```

### Test Chat Endpoint
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is inverse kinematics?", "mode": "normal"}'
```

### Test Streaming
```bash
curl -X POST http://localhost:8000/chat/stream \
  -H "Content-Type: application/json" \
  -d '{"query": "Explain bipedal robots", "mode": "normal"}'
```

## 📝 Configuration

See `app/config.py` for all configuration options. All settings are loaded from environment variables.

**Required Environment Variables:**
- `GEMINI_API_KEY` - Gemini 2.0 Flash API key
- `QDRANT_URL` - Qdrant Cloud cluster URL
- `QDRANT_API_KEY` - Qdrant API key
- `NEON_DATABASE_URL` - Neon Postgres connection string
- `CORS_ORIGINS` - Allowed CORS origins (comma-separated)

## 🔧 Development

### Run with Auto-Reload
```bash
cd app
uvicorn main:app --reload --port 8000
```

### Check Qdrant Collection
```python
from qdrant_client import QdrantClient

client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
info = client.get_collection("textbook")
print(f"Points: {info.points_count}")
```

## 📚 Architecture

### RAG Pipeline (Custom State Machine)

```
IDLE → RETRIEVING → GENERATING → STREAMING → COMPLETE
```

**States:**
1. **IDLE**: Waiting for query
2. **RETRIEVING**: Searching Qdrant for relevant chunks
3. **GENERATING**: Building prompt for Gemini
4. **STREAMING**: Streaming response tokens
5. **COMPLETE**: Response finished

### Normal Mode
- Query → Embed → Search Qdrant (top 5 chunks) → Gemini

### Highlight Mode (Zero-Leakage)
- Pre-filter Qdrant by page range
- Query → Embed → Search filtered set (top 3) → Gemini
- Ensures NO chunks outside selected pages

## 🚨 Troubleshooting

### "Missing required environment variables"
- Ensure `.env` file exists
- Check all required variables are set

### "Qdrant connection failed"
- Verify `QDRANT_URL` and `QDRANT_API_KEY`
- Check network connection to Qdrant Cloud

### "Gemini API error"
- Verify `GEMINI_API_KEY` is valid
- Check quota limits in Google AI Studio

### Ingestion takes too long
- PDF is large - this is normal for 100+ page books
- Expected: ~30 minutes for full textbook

## 📄 License

Part of Humanoid Robotics Textbook project (GIAIC 2025 Hackathon).

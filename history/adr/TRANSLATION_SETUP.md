# Translation Setup - Using Cohere Embeddings

## Overview
Translation ab **Cohere embeddings** ka data use karti hai jo Qdrant me stored hai. Yahi data chatbot bhi use karta hai.

## System Architecture

```
Book Content (Markdown)
    ↓
Cohere Embeddings (embed-english-v3.0)
    ↓
Qdrant Vector Database
    ↓
    ├─→ Chatbot (Q&A responses)
    └─→ Translation (Urdu translation)
```

## How It Works

1. **Book ka data already embedded hai** Qdrant me
2. **Translate button click karne pe:**
   - Current page ka URL detect hota hai
   - Qdrant se us page ke **embedded chunks fetch hote hain**
   - Cohere ka **command-r-08-2024 model** use karke translate hota hai
   - Result Urdu me display hota hai

## Setup Instructions

### 1. Chatbot Backend Start Karo (Port 8001)

```bash
cd rag-backend/chatbot
start-backend.bat
```

Ya manually:
```bash
cd rag-backend/chatbot
venv\Scripts\activate
python -m uvicorn app:app --host 0.0.0.0 --port 8001 --reload
```

### 2. Docusaurus Dev Server Start Karo (Port 3000)

```bash
cd my-website
npm start
```

### 3. Translation Use Karo

1. Browser me http://localhost:3000 kholo
2. Kisi bhi book page pe jao
3. Top right me **"🌐 اردو"** button click karo
4. Page content Urdu me translate ho jayega!

## Important Notes

- ✅ **Chatbot backend (8001) running hona chahiye** translation ke liye
- ✅ **Page content Qdrant me embedded hona chahiye**
- ✅ Translation **cache hoti hai** sessionStorage me (fast reload)
- ✅ English me wapas aane ke liye "English" button click karo

## API Endpoints

### Chatbot Backend (Port 8001)

- `GET /` - API info
- `GET /health` - Health check
- `POST /chat` - Chatbot Q&A (non-streaming)
- `POST /stream-chat` - Chatbot Q&A (streaming)
- **`POST /translate`** - Translation endpoint (NEW!)

### Translation Request Format

```json
{
  "page_url": "http://localhost:3000/docs/introduction",
  "target_language": "urdu"
}
```

### Translation Response Format

```json
{
  "success": true,
  "translation": "اردو میں ترجمہ شدہ متن...",
  "source": "cohere_embeddings"
}
```

## Benefits of Using Cohere Embeddings

1. **Consistency**: Same data jo chatbot use karta hai
2. **No Extra Embedding**: Data already Qdrant me hai
3. **Fast**: Direct vector database se fetch
4. **Accurate**: Book ka exact content translate hota hai
5. **Cost Effective**: Ek hi embedding reuse hoti hai

## Troubleshooting

### Error: "No content found for this page"
- Check ke page ka data Qdrant me embedded hai
- `ingest.py` run karo to embed new pages

### Error: "Translation failed"
- Chatbot backend (port 8001) chal raha hai check karo
- `.env` file me `COHERE_API_KEY` check karo

### Error: "Connection refused"
- Backend server start karo: `cd rag-backend/chatbot && start-backend.bat`

## Configuration Files

- **Backend**: `rag-backend/chatbot/app.py` - Translation endpoint
- **Frontend**: `my-website/src/theme/DocItem/Content/index.js` - Translation logic
- **Button**: `my-website/src/components/LanguageToggle/` - UI component
- **Config**: `rag-backend/chatbot/config.py` - Cohere & Qdrant settings

## Environment Variables

Backend `.env` file me required:
```
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
SITEMAP_URL=https://humanoid-robotics-textbook-psi.vercel.app/sitemap.xml
```

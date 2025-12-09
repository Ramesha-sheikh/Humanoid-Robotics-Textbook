import os
import sys
import re
from fastapi import FastAPI, Request, HTTPException
from fastapi.responses import StreamingResponse, HTMLResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import json
import asyncio

# Adjust the path to import from parent directories
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..'))

from app.agents.rag_agent import RAGAgent
from app.ingestion.ingest import ingest_documents
from app.config import settings

app = FastAPI(
    title="SpecKit Book RAG Chatbot API",
    description="API for the RAG-powered chatbot to answer questions from the AI book.",
    version="1.0.0",
)

# Configure CORS
origins = settings.BACKEND_CORS_ORIGINS.split(',')
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"] if "*" in origins else origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

rag_agent = RAGAgent()

class ChatRequest(BaseModel):
    user_query: str
    session_id: str | None = None
    user_id: str | None = None

class ChatResponseSource(BaseModel):
    title: str
    url: str
    heading: str | None = None

class ChatResponse(BaseModel):
    answer: str
    sources: List[ChatResponseSource]
    session_id: str | None = None

@app.get("/health", response_class=HTMLResponse)
async def health_check():
    return "<h1>RAG Chatbot Backend is Healthy</h1>"

@app.post("/api/chat") # Set response_model for non-streaming output structure
async def chat_endpoint(request: ChatRequest):
    async def generate_response():
        full_answer_chunks = []
        all_sources = []

        # Use a temporary list to collect chunks from the agent
        async for chunk in rag_agent.run_stream(request.user_query):
            full_answer_chunks.append(chunk)
            yield f"data: {json.dumps({"type": "text", "content": chunk})}\\n\\n"

        full_answer = "".join(full_answer_chunks)

        # Extract sources from the full_answer if they are embedded as markdown links
        # This is a placeholder for actual source extraction logic from tool outputs
        source_matches = re.findall(r'\[(.*?)\]\((.*?)\)', full_answer)
        for title, url in source_matches:
            # Further parsing for heading might be needed if embedded in URL or title
            all_sources.append(ChatResponseSource(title=title, url=url, heading=None))

        # Placeholder for final event, including collected sources and session_id
        final_event = {
            "type": "final",
            "answer": full_answer,
            "sources": [s.model_dump() for s in all_sources], # Use model_dump for Pydantic v2
            "session_id": request.session_id
        }
        yield f"data: {json.dumps(final_event)}\\n\\n"

    return StreamingResponse(generate_response(), media_type="text/event-stream")


@app.post("/api/ingest")
async def ingest_endpoint():
    # In a real application, you'd add authentication/authorization here
    # For this hackathon, we'll keep it simple.
    try:
        await ingest_documents()
        return {"status": "success", "message": "Document ingestion initiated successfully."}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Ingestion failed: {e}")
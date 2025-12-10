"""
FastAPI Backend for RAG Chatbot
Gemini 2.0 Flash + Qdrant + Neon Postgres
"""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
from typing import Optional, List, AsyncGenerator
import asyncio

from app.config import settings

# Initialize FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="Humanoid Robotics Textbook RAG Chatbot powered by Gemini 2.0 Flash",
    version="1.0.0"
)

# CORS Middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# Request/Response Models
class ChatRequest(BaseModel):
    query: str
    mode: str = "normal"  # "normal" or "highlight"
    highlight_context: Optional[dict] = None


class Source(BaseModel):
    chapter: str
    section: str
    page: int


class ChatResponse(BaseModel):
    answer: str
    sources: List[Source]
    latency_ms: int
    mode: str


# Health Check Endpoint
@app.get("/health")
async def health_check():
    """Health check endpoint for deployment monitoring."""
    return {
        "status": "ok",
        "version": "1.0.0",
        "service": "rag-chatbot-api"
    }


# Chat Endpoint (Non-Streaming)
@app.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """
    Process a chat query and return complete response.

    Args:
        request: ChatRequest with query, mode, and optional highlight_context

    Returns:
        ChatResponse with answer, sources, and metadata
    """
    try:
        # TODO: Implement RAG logic (will add in next steps)
        # For now, return a placeholder response
        return ChatResponse(
            answer=f"Echo: {request.query} (Mode: {request.mode})",
            sources=[
                Source(chapter="1", section="1.1", page=1)
            ],
            latency_ms=100,
            mode=request.mode
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


# Streaming Chat Endpoint (Server-Sent Events)
@app.post("/chat/stream")
async def chat_stream(request: ChatRequest):
    """
    Process a chat query and stream response tokens via SSE.

    Args:
        request: ChatRequest with query, mode, and optional highlight_context

    Returns:
        StreamingResponse with SSE format
    """
    async def generate_sse() -> AsyncGenerator[str, None]:
        """Generate Server-Sent Events for streaming response."""
        try:
            # TODO: Implement streaming RAG logic (will add in next steps)
            # For now, stream a placeholder response
            tokens = request.query.split()

            for i, token in enumerate(tokens):
                # Simulate streaming delay
                await asyncio.sleep(0.1)

                # SSE format: data: {...}\n\n
                yield f"data: {{\"token\": \"{token} \", \"done\": false}}\n\n"

            # Send completion event
            yield f"data: {{\"token\": \"\", \"done\": true}}\n\n"

        except Exception as e:
            yield f"data: {{\"error\": \"{str(e)}\", \"done\": true}}\n\n"

    return StreamingResponse(
        generate_sse(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
        }
    )


# Root Endpoint
@app.get("/")
async def root():
    """Root endpoint with API information."""
    return {
        "message": "RAG Chatbot API",
        "docs": "/docs",
        "health": "/health",
        "endpoints": {
            "chat": "POST /chat",
            "stream": "POST /chat/stream"
        }
    }


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=True
    )

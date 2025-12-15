from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional
import google.generativeai as genai
from cohere import Client as CohereClient
from qdrant_client import QdrantClient
from starlette.responses import StreamingResponse
import uvicorn
import json

app = FastAPI(title="Physical AI Textbook RAG Chatbot")

# Allow Docusaurus to connect
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

genai.configure(api_key="AIzaSyBgRzUEvO_TrRzvidMmFVwHnsPnJJaHvh4")
gemini_model = genai.GenerativeModel('gemini-2.0-flash')
# Cohere + Qdrant clients
cohere = CohereClient("4MLD6mHjzAos0nII5NTGhXKjNC85YwaBQeqYt6cw")
qdrant = QdrantClient(
    url="https://bef78ad5-f70b-4095-abe8-b2990f68edfb.us-east4-0.gcp.cloud.qdrant.io",
    api_key="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.sWvnfQ5qyoEx8ZUFz0yXS4769JGpf_YyZoKHr1Vmqxs"

)

class ChatRequest(BaseModel):
    question: str
    selected_text: Optional[str] = None

class ChatResponse(BaseModel):
    answer: str
    sources: List[str] = []

@app.get("/")
def home():
    return {"message": "Physical AI Textbook RAG API is LIVE!", "status": "ready"}

@app.post("/chat", response_model=ChatResponse)
async def rag_chat(request: ChatRequest):
    try:
        # If user highlighted text \u2192 use only that
        if request.selected_text and len(request.selected_text.strip()) > 20:
            context = request.selected_text.strip()
            sources = ["Selected text from book"]
        else:
            # Embed query
            emb = cohere.embed(
                texts=[request.question],
                model="embed-english-v3.0",
                input_type="search_query"
            ).embeddings[0]

            # Search Qdrant
            hits = qdrant.query_points(
                collection_name="Book",
                vector=emb,
                limit=4,
                score_threshold=0.7
            )

            if not hits:
                context = "No relevant content found in the book."
                sources = []
            else:
                context = "\n\n".join([h.payload.get("chunk_text", "") for h in hits])
                sources = list(set(h.payload.get("url", "Unknown") for h in hits))

        # Generate answer with Gemini
        response = gemini_model.generate_content(
            f"Answer based ONLY on this Physical AI textbook content:\n\n{context}\n\nQuestion: {request.question}",
            generation_config=genai.types.GenerationConfig(
                temperature=0.1,
                max_output_tokens=600
            )
        )

        return ChatResponse(
            answer=response.text.strip(),
            sources=sources
        )

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error: {str(e)}")

@app.post("/chat/stream")
async def rag_chat_stream(request: ChatRequest):
    try:
        # If user highlighted text → use only that
        if request.selected_text and len(request.selected_text.strip()) > 20:
            context = request.selected_text.strip()
            sources = ["Selected text from book"]
        else:
            # Embed query
            emb = cohere.embed(
                texts=[request.question],
                model="embed-english-v3.0",
                input_type="search_query"
            ).embeddings[0]

            # Search Qdrant
            hits = qdrant.query_points(
                collection_name="Book",
                vector=emb,
                limit=4,
                score_threshold=0.7
            )

            if not hits:
                context = "No relevant content found in the book."
                sources = []
            else:
                context = "\n\n".join([h.payload.get("chunk_text", "") for h in hits])
                sources = list(set(h.payload.get("url", "Unknown") for h in hits))

        # Generator to stream Gemini response
        async def generate_stream():
            full_response_content = ""
            async for chunk in gemini_model.generate_content(
                f"Answer based ONLY on this Physical AI textbook content:\n\n{context}\n\nQuestion: {request.question}",
                generation_config=genai.types.GenerationConfig(
                    temperature=0.1,
                    max_output_tokens=600
                ),
                stream=True
            ):
                token = chunk.text
                if token:
                    full_response_content += token
                    yield f"data: {json.dumps({'token': token, 'done': False})}\n\n"

            # After streaming is done, send the final response with sources
            final_response = ChatResponse(answer=full_response_content.strip(), sources=sources)
            yield f"data: {json.dumps({'done': True, 'response': final_response.model_dump_json()})}\n\n" # Use model_dump_json() for Pydantic v2

        return StreamingResponse(generate_stream(), media_type="text/event-stream")

    except Exception as e:
        import traceback
        traceback.print_exc()
        raise HTTPException(status_code=500, detail=f"Stream API error: {str(e)}")

if __name__ == "__main__":
    print("Starting RAG server on http://localhost:8001")
    uvicorn.run(app, host="0.0.0.0", port=8001)

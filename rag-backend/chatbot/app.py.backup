from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
from pathlib import Path
import json
import yaml
from cohere import Client as CohereClient
from qdrant_client import QdrantClient
from config import config

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

cohere = CohereClient(config.COHERE_API_KEY)
qdrant = QdrantClient(url=config.QDRANT_URL, api_key=config.QDRANT_API_KEY)

# Agents & Skills load (tumhare frontend se)
AGENTS = {}
SKILLS = {}

def load_agents_and_skills():
    global AGENTS, SKILLS
    agents_path = Path("frontend/public/agents-config.json")
    if not agents_path.exists():
        agents_path = Path("my-website/public/agents-config.json")
    if agents_path.exists():
        with open(agents_path, "r", encoding="utf-8") as f:
            data = json.load(f)
            AGENTS = {a["id"]: a for a in data.get("agents", [])}

    skills_dir = Path(".claude/skills")
    if skills_dir.exists():
        for skill_path in skills_dir.glob("*/SKILL.md"):
            try:
                content = skill_path.read_text(encoding="utf-8")
                parts = content.split("---", 2)
                if len(parts) >= 3:
                    meta = yaml.safe_load(parts[1])
                    template = parts[2].strip()
                    if meta and "name" in meta:
                        SKILLS[meta["name"]] = {
                            "name": meta["name"],
                            "prompt_template": template,
                        }
            except Exception as e:
                print(f"Skill error: {e}")

load_agents_and_skills()

class Query(BaseModel):
    question: str
    selected_text: str = ""
    agent_id: str | None = None

@app.get("/")
def root():
    return {
        "message": "RAG Chatbot API",
        "version": "1.0",
        "endpoints": {
            "health": "GET /health - Check API health",
            "chat": "POST /chat - Send a question to the chatbot (non-streaming)",
            "stream_chat": "POST /stream-chat - Send a question and get streaming response (SSE)",
        },
        "example_request": {
            "url": "/chat",
            "method": "POST",
            "body": {
                "question": "What is robotics?",
                "selected_text": "",
                "agent_id": None
            }
        }
    }

@app.get("/health")
def health():
    return {"status": "ok", "version": "1.0"}

@app.post("/chat")
def chat(q: Query):
    try:
        agent = AGENTS.get(q.agent_id) if q.agent_id else None

        if q.selected_text.strip():
            context = q.selected_text
            sources = []
            final_query = q.question or "Summarize this."
        else:
            embedding = cohere.embed(
                texts=[q.question],
                model="embed-english-v3.0",
                input_type="search_query",
            ).embeddings[0]

            # Use default vector (no 'using' parameter) since collection uses unnamed vector
            results = qdrant.query_points(
                collection_name=config.COLLECTION_NAME,
                query=embedding,
                limit=5,
                score_threshold=0.3,  # Lower threshold for better results
                with_payload=True,
            ).points

            if not results:
                return {"answer": "No relevant data found.", "sources": []}

            context_parts = [p.payload.get("chunk_text", "") for p in results]
            context = "\n\n".join(context_parts)
            sources = list(set(p.payload.get("url", "") for p in results))
            final_query = q.question

        if agent and agent.get("rag_enabled"):
            message = agent.get("prompt_template", "<RAG_TEXT>\n\n<QUERY>").replace("<RAG_TEXT>", context).replace("<QUERY>", final_query)
        else:
            message = f"Context:\n{context}\n\nQuestion: {final_query}"

        answer = cohere.chat(model="command-r-08-2024", message=message).text

        return {"answer": answer, "sources": sources}

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/stream-chat")
async def stream_chat(q: Query):
    async def generate():
        try:
            agent = AGENTS.get(q.agent_id) if q.agent_id else None

            if q.selected_text.strip():
                context = q.selected_text
                sources = []
                final_query = q.question or "Summarize this."
            else:
                embedding = cohere.embed(
                    texts=[q.question],
                    model="embed-english-v3.0",
                    input_type="search_query",
                ).embeddings[0]

                results = qdrant.query_points(
                    collection_name=config.COLLECTION_NAME,
                    query=embedding,
                    limit=5,
                    score_threshold=0.3,  # Lower threshold for better results
                    with_payload=True,
                ).points

                if not results:
                    yield f'data: {json.dumps({"done": True, "response": json.dumps({"answer": "No relevant data found.", "sources": []})})}\n\n'
                    return

                context_parts = [p.payload.get("chunk_text", "") for p in results]
                context = "\n\n".join(context_parts)
                sources = list(set(p.payload.get("url", "") for p in results))
                final_query = q.question

            if agent and agent.get("rag_enabled"):
                message = agent.get("prompt_template", "<RAG_TEXT>\n\n<QUERY>").replace("<RAG_TEXT>", context).replace("<QUERY>", final_query)
            else:
                message = f"Context:\n{context}\n\nQuestion: {final_query}"

            # Stream from Cohere
            full_answer = ""
            stream = cohere.chat_stream(model="command-r-08-2024", message=message)

            for event in stream:
                if event.event_type == "text-generation":
                    token = event.text
                    full_answer += token
                    yield f'data: {json.dumps({"token": token, "done": False})}\n\n'

            # Send final response with sources
            final_response = {"answer": full_answer, "sources": sources}
            yield f'data: {json.dumps({"done": True, "response": json.dumps(final_response)})}\n\n'

        except Exception as e:
            yield f'data: {json.dumps({"error": str(e), "done": True})}\n\n'

    return StreamingResponse(generate(), media_type="text/event-stream")

# Translation endpoint using Cohere embeddings
class TranslateRequest(BaseModel):
    page_url: str
    target_language: str = "urdu"

@app.post("/translate")
def translate_page(req: TranslateRequest):
    """
    Translate book page content to Urdu using Cohere embeddings data
    """
    try:
        # Extract page identifier from URL
        # Normalize URLs - could be localhost or production
        page_url_clean = req.page_url.strip()

        # Get the path part
        if "humanoid-robotics-textbook-psi.vercel.app" in page_url_clean:
            # Already full URL - use as is
            page_path = page_url_clean
        else:
            # localhost URL - convert to production URL format
            path_only = page_url_clean.replace("http://localhost:3000", "")
            path_only = path_only.replace("https://localhost:3000", "")
            # Ensure trailing slash for docs
            if path_only and not path_only.endswith('/'):
                path_only += '/'
            page_path = f"https://humanoid-robotics-textbook-psi.vercel.app{path_only}"

        print(f"Looking for content with URL: {page_path}")

        # Simple approach: Use semantic search with page title/URL as query
        # This is faster than scrolling all points
        # Extract last part of URL as search query
        page_name = page_path.split('/')[-2] if page_path.endswith('/') else page_path.split('/')[-1]
        search_query = page_name.replace('-', ' ')

        print(f"Search query: {search_query}")

        # Create embedding for the query
        query_embedding = cohere.embed(
            texts=[search_query],
            model="embed-english-v3.0",
            input_type="search_query",
        ).embeddings[0]

        # Search with high limit to get all chunks from this page
        search_results = qdrant.query_points(
            collection_name=config.COLLECTION_NAME,
            query=query_embedding,
            limit=50,  # Get up to 50 chunks
            score_threshold=0.3,  # Lower threshold
            with_payload=True,
        ).points

        # Filter to URL match (flexible - check if URL contains path or vice versa)
        all_points = []
        for p in search_results:
            url = p.payload.get("url", "")
            # Match if either URL contains the path, or exact match
            if url == page_path or page_path in url or url.endswith(page_path.split('/')[-2] + '/'):
                all_points.append(p)

        print(f"Found {len(all_points)} matching chunks (out of {len(search_results)} search results)")

        if not all_points:
            # If no exact matches, just use top semantic search results
            print("No exact URL matches, using semantic search results")
            all_points = search_results[:10]  # Use top 10 results

        if not all_points:
            return {"success": False, "error": f"No content found for page: {page_path} (searched: {search_query})"}

        # Collect all text chunks for this page
        page_content = "\n\n".join([p.payload.get("chunk_text", "") for p in all_points])

        if not page_content.strip():
            return {"success": False, "error": "Empty content"}

        # Translate using Cohere chat model
        translation_prompt = f"""You are a professional translator. Translate the following English text to natural, fluent Urdu.

IMPORTANT RULES:
- Maintain the original meaning and tone
- Use proper Urdu grammar and vocabulary
- Keep the text clear and readable
- Do not add or remove any information
- Only provide the translation, no explanations

Text to translate:

{page_content}"""

        # Use Cohere to translate
        response = cohere.chat(
            model="command-r-08-2024",
            message=translation_prompt,
            temperature=0.3  # Lower temperature for more consistent translation
        )

        translated_text = response.text

        return {
            "success": True,
            "translation": translated_text,
            "source": "cohere_embeddings"
        }

    except Exception as e:
        print(f"Translation error: {str(e)}")
        return {"success": False, "error": str(e)}






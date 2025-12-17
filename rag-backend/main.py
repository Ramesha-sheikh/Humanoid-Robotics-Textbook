from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from cohere import Client as CohereClient
from qdrant_client import QdrantClient, models
import uvicorn
import json
import yaml
from pathlib import Path
import traceback # Ensure this is at the top
import sys       # Ensure this is at the top
from dotenv import load_dotenv
import os
import re # For regex validation
import ast # For Python code validation

load_dotenv()

app = FastAPI(title="Physical AI Textbook RAG Chatbot")

# Allow Docusaurus to connect (enhanced CORS configuration)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with your frontend URL
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    allow_origin_regex=r"https?://localhost(:[0-9]+)?|https?://127\.0\.0\.1(:[0-9]+)?|.*\.vercel\.app|.*\.panaversity\.org"
)

# Credentials
COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = "book"

# Check if keys are available
if not COHERE_API_KEY or not QDRANT_URL or not QDRANT_API_KEY:
    print("Warning: API keys not found. Running in mock mode with limited functionality.")
    API_KEYS_AVAILABLE = False
else:
    API_KEYS_AVAILABLE = True

# Initialize clients with longer timeout
class MockCohereClient:
    def embed(self, texts, model, input_type):
        # Return mock embeddings
        class MockEmbeddingResult:
            def __init__(self):
                self.embeddings = [[0.1] * 1024]  # Mock embedding vector
        return MockEmbeddingResult()

    def chat(self, message, model, temperature, max_tokens):
        # Return mock response
        class MockResponse:
            def __init__(self):
                self.text = f"Mock response: {message[:100]}... (This is a mock response to avoid API token errors)"
        return MockResponse()

class MockQdrantClient:
    def search(self, collection_name, query_vector, limit, score_threshold, with_payload):
        # Return mock search results
        class MockHit:
            def __init__(self):
                self.payload = {
                    "chunk_text": "This is mock content from the textbook. In a real scenario, this would be retrieved from the vector database using your API keys.",
                    "url": "mock-url",
                    "title": "Mock Content"
                }
        return [MockHit() for _ in range(min(limit, 2))]  # Return up to 2 mock results
    def get_collection(self, collection_name):
        return True  # Mock successful collection check

# Test the API keys first, and use mock if they don't work
cohere_client = MockCohereClient()
qdrant_client = MockQdrantClient()

# Only try to initialize real clients if API_KEYS_AVAILABLE is True and we want to test them
if API_KEYS_AVAILABLE:
    try:
        # Test if the Cohere API key is valid
        test_client = CohereClient(COHERE_API_KEY)
        # Test embedding to verify the key works
        test_result = test_client.embed(
            texts=["test"],
            model="embed-english-v3.0",
            input_type="search_query"
        )
        # If we get here, the key works, so use real clients
        cohere_client = test_client

        # Test if Qdrant connection works
        test_qdrant = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY, timeout=60.0)
        test_qdrant.get_collection(COLLECTION_NAME)  # This will raise an exception if connection fails
        qdrant_client = test_qdrant
        print("Successfully connected to Cohere and Qdrant APIs")
    except Exception as e:
        print(f"API connection failed (possibly due to invalid keys): {e}")
        print("Using mock clients for limited functionality")
        # Keep the mock clients

# --- Load Agent and Skill Definitions ---
AGENTS_CONFIG: Dict[str, Any] = {}
SKILLS_CONFIG: Dict[str, Any] = {}

def load_agent_configs():
    global AGENTS_CONFIG
    try:
        # First try the original path (for different deployment configurations)
        config_path = Path(__file__).parent.parent / "frontend" / "public" / "agents-config.json"
        if not config_path.exists():
            # Try the my-website path (more common in this project structure)
            config_path = Path(__file__).parent.parent / "my-website" / "public" / "agents-config.json"

        if config_path.exists():
            with open(config_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            AGENTS_CONFIG = {agent["id"]: agent for agent in data.get("agents", [])}
            print(f"Loaded {len(AGENTS_CONFIG)} agent configurations.")
        else:
            print(f"Warning: agents-config.json not found at {config_path}. Using fallback agent.")
            AGENTS_CONFIG = {
                "teaching-assistant": {
                    "id": "teaching-assistant",
                    "name": "Teaching Assistant",
                    "greeting": "Hi! I'm your tutor!",
                    "prompt": "You are a friendly teaching assistant. Answer clearly using book content."
                }
            }
    except Exception as e:
        print(f"Error loading agents-config.json: {e}. Using fallback agent.")
        AGENTS_CONFIG = {"teaching-assistant": {"id": "teaching-assistant", "name": "Teaching Assistant", "greeting": "Hi! I'm your tutor!", "prompt": "You are a friendly teaching assistant. Answer clearly using book content."}}


def load_skill_configs():
    global SKILLS_CONFIG
    skills_dir = Path(__file__).parent.parent / ".claude" / "skills"
    if not skills_dir.is_dir():
        print(f"Error: Skills directory not found at {skills_dir}")
        return

    for skill_dir in skills_dir.iterdir():
        if skill_dir.is_dir():
            skill_md_path = skill_dir / "SKILL.md"
            if skill_md_path.is_file():
                try:
                    content = skill_md_path.read_text(encoding="utf-8")
                    # Split YAML frontmatter from markdown content
                    parts = content.split("---", 2)
                    if len(parts) < 3:
                        print(f"Warning: SKILL.md for {skill_dir.name} is missing YAML frontmatter.")
                        continue

                    frontmatter = yaml.safe_load(parts[1])
                    if frontmatter and "name" in frontmatter:
                        SKILLS_CONFIG[frontmatter["name"]] = frontmatter
                        # Add full markdown content if needed later (not used currently, but good to have)
                        SKILLS_CONFIG[frontmatter["name"]]["full_content"] = parts[2].strip()
                except Exception as e:
                    print(f"Error loading SKILL.md for {skill_dir.name}: {e}")
    print(f"Loaded {len(SKILLS_CONFIG)} skill configurations.")

# Load configs on startup
load_agent_configs()
load_skill_configs()

class ChatRequest(BaseModel):
    question: str
    selected_text: Optional[str] = None
    agent_id: Optional[str] = None # Added agent_id

class ChatResponse(BaseModel):
    answer: str
    sources: List[str] = []

# --- Quiz Format Validation ---
def validate_quiz_format(text: str) -> bool:
    """
    Validates if the text adheres to the expected MCQ format.
    Checks for:
    - At least one question number (e.g., "1.")
    - At least one option (e.g., "a)")
    - "Correct Answer:" presence
    - "Explanation:" presence
    """
    has_question_number = bool(re.search(r"^\d+\.\s", text, re.MULTILINE))
    has_option = bool(re.search(r"^[a-dA-D]\)", text, re.MULTILINE))
    has_correct_answer = "Correct Answer:" in text
    has_explanation = "Explanation:" in text
    
    return has_question_number and has_option and has_correct_answer and has_explanation

# --- Python Code Validation ---
def validate_python_code(code: str) -> Optional[str]:
    """
    Checks Python code for basic syntax errors.
    Returns error message if invalid, None if valid.
    """
    try:
        # Extract code if it's in a markdown block
        code_match = re.search(r"```python\n(.*?)```", code, re.DOTALL)
        if code_match:
            code_to_check = code_match.group(1).strip()
        else:
            code_to_check = code.strip()

        if not code_to_check:
            return "No Python code found to validate."
        
        ast.parse(code_to_check)
        return None # No syntax errors
    except SyntaxError as e:
        return f"Syntax Error: {e}"
    except Exception as e:
        return f"An unexpected error occurred during code validation: {e}"

# --- FastAPI Endpoints ---
@app.get("/")
def home():
    return {"message": "Physical AI Textbook RAG API is LIVE!", "status": "ready"}

@app.get("/health")
def health_check():
    return {"status": "healthy", "api_keys_available": API_KEYS_AVAILABLE}

from fastapi.responses import StreamingResponse
import json

@app.post("/chat", response_model=ChatResponse)
async def rag_chat(request: ChatRequest):
    try:
        # Debug prints
        print(f"DEBUG: AGENTS_CONFIG keys: {AGENTS_CONFIG.keys()}")
        print(f"DEBUG: SKILLS_CONFIG keys: {SKILLS_CONFIG.keys()}")
        print(f"DEBUG: Request agent_id: {request.agent_id}")

        # Choose agent (fallback to teaching-assistant)
        current_agent_id = request.agent_id if request.agent_id and request.agent_id in AGENTS_CONFIG else "teaching-assistant"
        agent_config = AGENTS_CONFIG.get(current_agent_id)
        
        if not agent_config:
            # This should ideally not happen with fallback, but as a safeguard
            raise HTTPException(status_code=400, detail=f"Agent '{current_agent_id}' not found in configurations.")

        context = ""
        sources = []
        base_prompt_instruction = agent_config["prompt"] # Use agent's base prompt

        # If user highlighted text → use only that
        if request.selected_text and len(request.selected_text.strip()) > 20:
            context = request.selected_text.strip()
            sources = ["Selected text from book"]
        else:
            try:
                if API_KEYS_AVAILABLE:
                    # Embed query using real API
                    emb = cohere_client.embed(
                        texts=[request.question],
                        model="embed-english-v3.0",
                        input_type="search_query"
                    ).embeddings[0]

                    # Search Qdrant using client.search (reverted to previous working method)
                    hits = qdrant_client.search(
                        collection_name=COLLECTION_NAME,
                        query_vector=emb,
                        limit=4,
                        score_threshold=0.7,
                        with_payload=True
                    )

                    if hits:
                        context = "\n\n".join([h.payload.get("chunk_text", "") for h in hits])
                        sources = list(set(h.payload.get("url", "Unknown") for h in hits))
                    else:
                        context = "No matching content found in the book."
                        sources = []
                else:
                    # Mock search functionality when API keys aren't available
                    print("Using mock search functionality")
                    context = "This is mock content from the textbook. In a real scenario, this would be retrieved from the vector database using your API keys. The system would search for relevant content based on your question and return the most relevant textbook chapters or sections."
                    sources = ["mock-source-1", "mock-source-2"]
            except Exception as e:
                context = "Search temporarily unavailable. Please try again later."
                sources = []
                print(f"Error during search: {e}")
                traceback.print_exc()
        
        # --- Dynamic Prompt Construction based on Agent/Skills ---
        final_prompt_message = ""
        skill_name_triggered = None

        question_lower = request.question.lower()
        base_prompt_lower = base_prompt_instruction.lower()

        # Prioritize skill selection based on agent's prompt hints and keywords in question
        if "robotics-code-generator" in base_prompt_lower or "code" in question_lower or "generate" in question_lower:
            skill_name_triggered = "robotics-code-generator"
        elif "quiz" in base_prompt_lower or "quiz" in question_lower or "mcq" in question_lower:
            skill_name_triggered = "quiz-maker-robotics" # Using quiz-maker-robotics as per latest
        elif "summarize" in base_prompt_lower or "summarize" in question_lower or "overview" in question_lower:
            skill_name_triggered = "chapter-summarizer"
        elif "explain" in base_prompt_lower or "what is" in question_lower or "concept" in question_lower:
            skill_name_triggered = "concept-explainer"
        elif "lesson" in base_prompt_lower or "lesson" in question_lower:
            skill_name_triggered = "lesson-builder"
        
        # --- Cohere Generation with Self-Correction ---
        generated_answer = ""
        final_sources = list(set(sources)) # Ensure sources are unique and ready

        retries = 0
        MAX_RETRIES = 2
        
        while retries <= MAX_RETRIES:
            if skill_name_triggered and skill_name_triggered in SKILLS_CONFIG:
                skill_used = SKILLS_CONFIG.get(skill_name_triggered)
                if skill_used:
                    current_prompt_for_llm = ""
                    # Dynamically replace placeholder based on skill type
                    if "<TASK>" in skill_used["prompt"]:
                        current_prompt_for_llm = skill_used["prompt"].replace("<TASK>", request.question)
                    elif "<TEXT>" in skill_used["prompt"]:
                        current_prompt_for_llm = skill_used["prompt"].replace("<TEXT>", context if context != "No matching content found in the book." else request.question)
                    elif "<CONCEPT>" in skill_used["prompt"]:
                        current_prompt_for_llm = skill_used["prompt"].replace("<CONCEPT>", request.question)
                    
                    # Append output format instruction if available
                    if skill_used.get("output_format"):
                        # Special handling for quiz-maker-robotics to make it very explicit
                        if skill_name_triggered == "quiz-maker-robotics":
                            correction_instruction = ""
                            if retries > 0:
                                correction_instruction = "\n\nCRITICAL: Your previous response did NOT meet the required quiz format. You MUST provide 4 MCQs formatted exactly as requested. Ensure each question has 4 options (a,b,c,d), a 'Correct Answer:' line, and an 'Explanation:' line. DO NOT deviate from this structure."
                            
                            current_prompt_for_llm = f"{{{skill_used['prompt'].replace('<TEXT>', context if context != 'No matching content found in the book.' else request.question)}}}{correction_instruction}\n\nPlease format the output as a numbered list of multiple-choice questions.\nFor each question:\n1.  Provide the question text.\n2.  List 4 options, labeled a), b), c), and d).\n3.  Clearly state the \"Correct Answer:\" followed by the letter and option.\n4.  Provide an \"Explanation:\" for the correct answer.\n\nExample Format:\n1. Question text?\n   a) Option A\n   b) Option B\n   c) Option C\n   d) Option D\nCorrect Answer: b) Option B\nExplanation: [Detailed explanation here]\n"
                        elif skill_name_triggered == "robotics-code-generator":
                            # For code generation, add self-correction instruction
                            error_feedback = ""
                            if retries > 0:
                                error_feedback = "\n\nCRITICAL: Your previous code had the following error. Please fix it:\n" + generated_answer # generated_answer holds the previous error
                            current_prompt_for_llm = f"{skill_used['prompt'].replace('<TASK>', request.question)}{error_feedback}" + "\n\nOutput ONLY the corrected code in a markdown block. Do NOT include any explanations outside the code block."
                        else:
                            current_prompt_for_llm += f"\n\nOutput in {skill_used['output_format']} format."
                else: # Fallback if skill definition is somehow missing after lookup
                    current_prompt_for_llm = f"{base_prompt_instruction}\n\nBook Content:\n{context}\n\nQuestion: {request.question}"
            else:
                # Fallback to general RAG prompt if no specific skill is triggered or skill not found
                current_prompt_for_llm = f"{base_prompt_instruction}\n\nBook Content:\n{context}\n\nQuestion: {request.question}"

            # Generate answer with Cohere
            response = cohere_client.chat(
                message=current_prompt_for_llm,
                model="command-r-08-2024", 
                temperature=0.1,
                max_tokens=1500
            )
            generated_answer = response.text.strip()

            # --- Self-Correction Validation ---
            if skill_name_triggered == "quiz-maker-robotics":
                if validate_quiz_format(generated_answer):
                    print(f"DEBUG: Quiz format validated successfully after {retries} retries.")
                    return ChatResponse(answer=generated_answer, sources=final_sources)
                else:
                    print(f"DEBUG: Quiz format validation FAILED. Retrying... (Retry {retries + 1}/{MAX_RETRIES})")
                    retries += 1
                    # If retries are exhausted, break and return the last generated (failed) answer
                    if retries > MAX_RETRIES:
                        print("DEBUG: Max retries for quiz format exceeded.")
                        break # Exit loop to return current (failed) answer
                    # If retrying, generated_answer (from failed attempt) is used as feedback in next loop iteration
            elif skill_name_triggered == "robotics-code-generator":
                code_validation_error = validate_python_code(generated_answer)
                if code_validation_error is None:
                    print(f"DEBUG: Python code validated successfully after {retries} retries.")
                    return ChatResponse(answer=generated_answer, sources=final_sources)
                else:
                    print(f"DEBUG: Python code validation FAILED: {code_validation_error}. Retrying... (Retry {retries + 1}/{MAX_RETRIES})")
                    retries += 1
                    # Prepare for retry: the generated_answer in this case is the code with syntax error,
                    # so we prepend the error to the message for Cohere to fix it.
                    current_prompt_for_llm += f"\n\nCRITICAL: Your previous code had the following error. Please fix it:\n{code_validation_error}\n\nPrevious Code:\n{generated_answer}\n\nOutput ONLY the corrected code in a markdown block. Do NOT include any explanations outside the code block."
                    if retries > MAX_RETRIES:
                        print("DEBUG: Max retries for code validation exceeded. Returning last attempt with error.")
                        generated_answer = f"I tried to generate the code multiple times but encountered errors. Last attempt's error: {code_validation_error}\n\nPrevious Code:\n{generated_answer}"
                        break
            else:
                # For non-quiz, non-code skills, no special format validation needed for now
                return ChatResponse(answer=generated_answer, sources=final_sources)
        
        # If loop finishes (e.g., max retries for quiz or code), return the last generated answer (which might be an error msg)
        return ChatResponse(answer=generated_answer, sources=final_sources)

    except Exception as e:
        import sys # Ensure sys is available
        print(f"ERROR in rag_chat: {e}")
        print(f"DEBUG: Exception details: {sys.exc_info()}")
        # Return a more frontend-friendly error response
        return ChatResponse(
            answer="I'm sorry, I'm having trouble processing your request right now. Please try again later.",
            sources=[]
        )

@app.post("/api/chat", response_model=ChatResponse)
async def rag_chat_api(request: ChatRequest):
    # Call the same chat function for API compatibility
    return await rag_chat(request)

@app.post("/stream-chat")
async def stream_chat(request: ChatRequest):
    # Create a streaming response for the frontend
    async def generate():
        try:
            # Get the response from the standard chat function
            response = await rag_chat(request)

            # Stream the response word by word to simulate token streaming
            words = response.answer.split()
            accumulated_text = ""

            for i, word in enumerate(words):
                accumulated_text += (" " if i > 0 else "") + word
                # Stream partial response with token
                data = {
                    "token": word + (" " if i < len(words) - 1 else ""),
                    "done": False,
                    "response": None  # Not done yet
                }
                yield f"data: {json.dumps(data)}\n\n"

            # Finally, send the complete response
            final_data = {
                "token": "",
                "done": True,
                "response": {
                    "answer": response.answer,
                    "sources": response.sources
                }
            }
            yield f"data: {json.dumps(final_data)}\n\n"
            yield "data: [DONE]\n\n"
        except Exception as e:
            print(f"ERROR in stream_chat: {e}")
            error_data = {
                "done": True,
                "error": "I'm sorry, I'm having trouble processing your request right now. Please try again later."
            }
            yield f"data: {json.dumps(error_data)}\n\n"

    return StreamingResponse(generate(), media_type="text/event-stream")

# Additional endpoint that might be expected by the frontend
@app.post("/api/stream")
async def stream_api(request: ChatRequest):
    # Create a streaming response with different format
    async def generate():
        try:
            response = await rag_chat(request)
            # Stream the response word by word to simulate token streaming
            words = response.answer.split()

            for i, word in enumerate(words):
                # Stream partial response
                yield f"data: {json.dumps({'type': 'text', 'content': word + (' ' if i < len(words) - 1 else '')})}\n\n"

            # Send sources
            yield f"data: {json.dumps({'type': 'sources', 'content': response.sources})}\n\n"
            yield "data: [DONE]\n\n"
        except Exception as e:
            print(f"ERROR in stream_api: {e}")
            yield f"data: {json.dumps({'type': 'error', 'content': 'Connection error'})}\n\n"

    return StreamingResponse(generate(), media_type="text/event-stream")

if __name__ == "__main__":
    print("Starting RAG server on http://localhost:8001")
    uvicorn.run(app, host="0.0.0.0", port=8001)
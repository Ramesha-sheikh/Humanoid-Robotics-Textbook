"""
RAG (Retrieval-Augmented Generation) System
Using Gemini 2.0 Flash + Qdrant + Custom State Machine
"""

import google.generativeai as genai
from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, AsyncGenerator, Optional
from enum import Enum
import time

from app.config import settings

# Configure Gemini API
genai.configure(api_key=settings.gemini_api_key)

# Initialize Qdrant Client
qdrant_client = QdrantClient(
    url=settings.qdrant_url,
    api_key=settings.qdrant_api_key,
)

# Collection name
COLLECTION_NAME = "textbook"


# State Machine for RAG Pipeline
class ChatState(Enum):
    """Simple state machine replacing LangGraph."""
    IDLE = "idle"
    RETRIEVING = "retrieving"
    GENERATING = "generating"
    STREAMING = "streaming"
    COMPLETE = "complete"
    ERROR = "error"


# Data Models
class Chunk:
    """Represents a textbook chunk with metadata."""
    def __init__(self, text: str, chapter: str, section: str, page: int):
        self.text = text
        self.chapter = chapter
        self.section = section
        self.page = page


class HighlightContext:
    """Context for Highlight Mode queries."""
    def __init__(self, text: str, start_page: int, end_page: int, chapter_slug: str):
        self.text = text
        self.start_page = start_page
        self.end_page = end_page
        self.chapter_slug = chapter_slug


# Retrieval Functions
async def retrieve_normal_mode(query: str, limit: int = 5) -> List[Chunk]:
    """
    Retrieve chunks from entire textbook (Normal Mode).

    Args:
        query: User's question
        limit: Number of chunks to retrieve

    Returns:
        List of relevant chunks with metadata
    """
    # TODO: Implement embedding + Qdrant search
    # For now, return placeholder
    return [
        Chunk(
            text="Inverse kinematics is the process of determining joint angles...",
            chapter="kinematics",
            section="5.2",
            page=42
        )
    ]


async def retrieve_highlight_mode(
    query: str,
    highlight: HighlightContext,
    limit: int = 3
) -> List[Chunk]:
    """
    Retrieve chunks ONLY from highlighted text range (Highlight Mode).
    Zero-leakage: Pre-filters by metadata before retrieval.

    Args:
        query: User's question
        highlight: Highlight context with page range
        limit: Number of chunks to retrieve from filtered set

    Returns:
        List of chunks within page range only
    """
    # Metadata filter for zero-leakage
    filter_condition = models.Filter(
        must=[
            models.FieldCondition(
                key="chapter",
                match=models.MatchValue(value=highlight.chapter_slug)
            ),
            models.FieldCondition(
                key="page",
                range=models.Range(
                    gte=highlight.start_page,
                    lte=highlight.end_page
                )
            )
        ]
    )

    # TODO: Implement filtered search
    # For now, return placeholder
    return [
        Chunk(
            text=f"Content from pages {highlight.start_page}-{highlight.end_page}...",
            chapter=highlight.chapter_slug,
            section="N/A",
            page=highlight.start_page
        )
    ]


# Prompt Building
def build_prompt(query: str, chunks: List[Chunk], mode: str = "normal") -> str:
    """
    Build prompt for Gemini with retrieved context.

    Args:
        query: User's question
        chunks: Retrieved textbook chunks
        mode: "normal" or "highlight"

    Returns:
        Formatted prompt string
    """
    context_parts = []

    for i, chunk in enumerate(chunks, 1):
        context_parts.append(
            f"[Chunk {i}] (Chapter: {chunk.chapter}, "
            f"Section: {chunk.section}, Page: {chunk.page})\n"
            f"{chunk.text}\n"
        )

    context = "\n".join(context_parts)

    prompt = f"""You are an expert on Humanoid Robotics. Answer the question based ONLY on the textbook excerpts below.

TEXTBOOK EXCERPTS:
{context}

QUESTION: {query}

ANSWER: Provide a clear, concise answer with citations in format [Chapter X, Section Y, Page Z]. If the excerpts don't contain enough information, say "I cannot find sufficient information in the provided context."
"""

    return prompt


# Gemini Generation (Streaming)
async def gemini_generate_stream(prompt: str) -> AsyncGenerator[str, None]:
    """
    Generate streaming response using Gemini 2.0 Flash.

    Args:
        prompt: Formatted prompt with context

    Yields:
        Response tokens as they're generated
    """
    try:
        # Initialize Gemini model
        model = genai.GenerativeModel("gemini-2.0-flash-exp")

        # Generate content with streaming
        response = model.generate_content(
            prompt,
            stream=True,
            generation_config=genai.types.GenerationConfig(
                temperature=0.7,
                max_output_tokens=1024,
            )
        )

        # Stream tokens
        for chunk in response:
            if hasattr(chunk, 'text'):
                yield chunk.text

    except Exception as e:
        yield f"\n\n[Error: {str(e)}]"


# Main RAG Pipeline
async def rag_query(
    query: str,
    mode: str = "normal",
    highlight_context: Optional[Dict] = None
) -> AsyncGenerator[str, None]:
    """
    Complete RAG pipeline with custom state machine.

    Args:
        query: User's question
        mode: "normal" or "highlight"
        highlight_context: Optional dict with highlight metadata

    Yields:
        Streaming response tokens
    """
    state = ChatState.IDLE
    start_time = time.time()

    try:
        # State: RETRIEVING
        state = ChatState.RETRIEVING

        if mode == "highlight" and highlight_context:
            highlight = HighlightContext(
                text=highlight_context.get("text", ""),
                start_page=highlight_context.get("startPage", 1),
                end_page=highlight_context.get("endPage", 1),
                chapter_slug=highlight_context.get("chapterSlug", "")
            )
            chunks = await retrieve_highlight_mode(query, highlight)
        else:
            chunks = await retrieve_normal_mode(query)

        # State: GENERATING
        state = ChatState.GENERATING
        prompt = build_prompt(query, chunks, mode)

        # State: STREAMING
        state = ChatState.STREAMING

        async for token in gemini_generate_stream(prompt):
            yield token

        # State: COMPLETE
        state = ChatState.COMPLETE

        elapsed = int((time.time() - start_time) * 1000)
        yield f"\n\n[Completed in {elapsed}ms]"

    except Exception as e:
        state = ChatState.ERROR
        yield f"\n\n[Error in {state.value} state: {str(e)}]"

"""
PDF Ingestion Pipeline
Textbook PDF → Chunks → Embeddings → Qdrant Cloud

Usage:
    python scripts/ingestion.py --pdf path/to/textbook.pdf
"""

import argparse
import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.append(str(Path(__file__).parent.parent))

import fitz  # PyMuPDF
import google.generativeai as genai
from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Tuple, Dict
import time

from app.config import settings

# Configure APIs
genai.configure(api_key=settings.gemini_api_key)
qdrant_client = QdrantClient(
    url=settings.qdrant_url,
    api_key=settings.qdrant_api_key,
)

COLLECTION_NAME = "textbook"
EMBEDDING_MODEL = "models/text-embedding-004"
CHUNK_SIZE = 512  # tokens
CHUNK_OVERLAP = 50  # tokens


def extract_text_from_pdf(pdf_path: str) -> List[Tuple[int, str]]:
    """
    Extract text from PDF preserving page numbers.

    Args:
        pdf_path: Path to PDF file

    Returns:
        List of (page_num, text) tuples
    """
    print(f"📄 Opening PDF: {pdf_path}")

    doc = fitz.open(pdf_path)
    pages = []

    for page_num in range(len(doc)):
        page = doc[page_num]
        text = page.get_text()
        pages.append((page_num + 1, text))  # 1-indexed pages

        if (page_num + 1) % 10 == 0:
            print(f"   Extracted {page_num + 1}/{len(doc)} pages...")

    print(f"✅ Extracted {len(doc)} pages")
    return pages


def chunk_text(pages: List[Tuple[int, str]], chunk_size: int, overlap: int) -> List[Dict]:
    """
    Split text into chunks with metadata.

    Args:
        pages: List of (page_num, text) tuples
        chunk_size: Target chunk size in characters (approximation for tokens)
        overlap: Overlap between chunks in characters

    Returns:
        List of chunks with metadata
    """
    print(f"✂️  Chunking text (size={chunk_size}, overlap={overlap})...")

    chunks = []

    for page_num, text in pages:
        # Simple chunking by character count (TODO: use tiktoken for accurate token count)
        text_length = len(text)
        start = 0

        while start < text_length:
            end = start + chunk_size
            chunk_text = text[start:end]

            # Skip empty chunks
            if chunk_text.strip():
                chunks.append({
                    "text": chunk_text,
                    "page": page_num,
                    "chapter": "unknown",  # TODO: detect chapter from content
                    "section": "unknown",   # TODO: detect section from content
                    "book_title": "Humanoid Robotics Textbook"
                })

            start += (chunk_size - overlap)

    print(f"✅ Created {len(chunks)} chunks")
    return chunks


def generate_embeddings(texts: List[str], batch_size: int = 100) -> List[List[float]]:
    """
    Generate embeddings using Google text-embedding-004.

    Args:
        texts: List of text chunks
        batch_size: Number of texts to embed per API call

    Returns:
        List of embedding vectors
    """
    print(f"🧠 Generating embeddings (batch_size={batch_size})...")

    embeddings = []

    for i in range(0, len(texts), batch_size):
        batch = texts[i:i + batch_size]

        # Generate embeddings for batch
        result = genai.embed_content(
            model=EMBEDDING_MODEL,
            content=batch,
            task_type="retrieval_document"
        )

        embeddings.extend(result['embedding'])

        print(f"   Embedded {min(i + batch_size, len(texts))}/{len(texts)} chunks...")

    print(f"✅ Generated {len(embeddings)} embeddings")
    return embeddings


def create_collection():
    """Create Qdrant collection if it doesn't exist."""
    print(f"📦 Creating collection '{COLLECTION_NAME}'...")

    try:
        qdrant_client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=models.VectorParams(
                size=768,  # text-embedding-004 dimension
                distance=models.Distance.COSINE
            )
        )
        print(f"✅ Collection '{COLLECTION_NAME}' created")
    except Exception as e:
        if "already exists" in str(e).lower():
            print(f"ℹ️  Collection '{COLLECTION_NAME}' already exists")
        else:
            raise


def upload_to_qdrant(chunks: List[Dict], embeddings: List[List[float]]):
    """
    Upload chunks and embeddings to Qdrant Cloud.

    Args:
        chunks: List of chunk metadata
        embeddings: List of embedding vectors
    """
    print(f"☁️  Uploading to Qdrant Cloud...")

    points = []
    for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
        points.append(
            models.PointStruct(
                id=i,
                vector=embedding,
                payload=chunk
            )
        )

    # Upload in batches
    batch_size = 100
    for i in range(0, len(points), batch_size):
        batch = points[i:i + batch_size]
        qdrant_client.upsert(
            collection_name=COLLECTION_NAME,
            points=batch
        )
        print(f"   Uploaded {min(i + batch_size, len(points))}/{len(points)} points...")

    print(f"✅ Uploaded {len(points)} points to Qdrant")


def main():
    """Main ingestion pipeline."""
    parser = argparse.ArgumentParser(description="Ingest textbook PDF into Qdrant")
    parser.add_argument("--pdf", required=True, help="Path to textbook PDF file")
    args = parser.parse_args()

    start_time = time.time()

    try:
        # Step 1: Extract text from PDF
        pages = extract_text_from_pdf(args.pdf)

        # Step 2: Chunk text
        chunks = chunk_text(pages, chunk_size=CHUNK_SIZE * 4, overlap=CHUNK_OVERLAP * 4)  # char approximation

        # Step 3: Generate embeddings
        texts = [chunk["text"] for chunk in chunks]
        embeddings = generate_embeddings(texts)

        # Step 4: Create collection
        create_collection()

        # Step 5: Upload to Qdrant
        upload_to_qdrant(chunks, embeddings)

        elapsed = int(time.time() - start_time)
        print(f"\n🎉 Ingestion complete in {elapsed} seconds!")
        print(f"   - {len(pages)} pages processed")
        print(f"   - {len(chunks)} chunks created")
        print(f"   - {len(embeddings)} embeddings generated")
        print(f"   - Uploaded to Qdrant collection: {COLLECTION_NAME}")

    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()

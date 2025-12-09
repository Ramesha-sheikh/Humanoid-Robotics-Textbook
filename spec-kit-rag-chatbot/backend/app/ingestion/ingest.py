import asyncio
import os
import sys

# Adjust the path to import from parent directories
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..'))

from app.ingestion.loader import DocumentLoader
from app.embedding.cohere_embedder import get_cohere_embeddings
from app.vector.qdrant_client import QdrantVectorDB, COLLECTION_NAME

async def ingest_documents():
    print("Starting document ingestion...")

    # 1. Load documents
    # Assuming the 'docs' folder is at the project root level
    current_dir = os.path.dirname(os.path.abspath(__file__))
    docs_path = os.path.join(current_dir, "..", "..", "..", "..", "docs")

    if not os.path.exists(docs_path):
        print(f"Error: Docusaurus docs folder not found at {docs_path}")
        print("Please ensure your MDX book content is in a 'docs' folder at the project root.")
        return

    loader = DocumentLoader(docs_base_path=docs_path, chunk_size=600, chunk_overlap=100)
    documents = loader.load_documents()
    print(f"Loaded {len(documents)} documents.")

    all_chunks = []
    for doc in documents:
        chunks = loader.chunk_document(doc)
        all_chunks.extend(chunks)
    print(f"Generated {len(all_chunks)} chunks from all documents.")

    if not all_chunks:
        print("No chunks generated. Exiting ingestion.")
        return

    # 2. Generate embeddings for all chunks
    chunk_texts = [chunk['text'] for chunk in all_chunks]
    print(f"Generating embeddings for {len(chunk_texts)} chunks...")
    embeddings = await get_cohere_embeddings(chunk_texts)
    print("Embeddings generated.")

    # 3. Upsert into Qdrant
    qdrant_db = QdrantVectorDB()
    await qdrant_db.recreate_collection() # Start with a fresh collection
    await qdrant_db.upsert_chunks(all_chunks, embeddings)
    print("Document ingestion completed successfully!")

if __name__ == "__main__":
    # Run the ingestion process
    asyncio.run(ingest_documents())

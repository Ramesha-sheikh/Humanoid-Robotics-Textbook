import os
from typing import List, Dict, Any
from qdrant_client import QdrantClient, models
from dotenv import load_dotenv

load_dotenv()

QDRANT_HOST = os.getenv("QDRANT_HOST", "localhost")
QDRANT_PORT = int(os.getenv("QDRANT_PORT", 6333))

COLLECTION_NAME = "book_rag_chunks"
VECTOR_SIZE = 1024 # Dimension for Cohere's embed-multilingual-v3.0

class QdrantVectorDB:
    def __init__(self):
        self.client = QdrantClient(host=QDRANT_HOST, port=QDRANT_PORT)

    async def recreate_collection(self):
        """
        Deletes and recreates the Qdrant collection.
        """
        print(f"Recreating collection: {COLLECTION_NAME}")
        await self.client.recreate_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=models.VectorParams(size=VECTOR_SIZE, distance=models.Distance.COSINE),
        )
        print(f"Collection {COLLECTION_NAME} recreated successfully.")

    async def upsert_chunks(self, chunks: List[Dict[str, Any]], embeddings: List[List[float]]):
        """
        Upserts a list of chunks and their corresponding embeddings into the Qdrant collection.

        Args:
            chunks: A list of dictionaries, where each dictionary represents a chunk with metadata.
            embeddings: A list of embedding vectors corresponding to the chunks.
        """
        points = []
        for i, chunk in enumerate(chunks):
            points.append(models.PointStruct(
                id=chunk["chunk_id"], # Use chunk_id as the Qdrant point ID
                vector=embeddings[i],
                payload=chunk # Store the entire chunk dictionary as payload
            ))

        print(f"Upserting {len(points)} points into collection {COLLECTION_NAME}...")
        operation_info = await self.client.upsert(
            collection_name=COLLECTION_NAME,
            wait=True,
            points=points
        )
        print(f"Upsert operation completed: {operation_info}")

    async def search_chunks(self, query_embedding: List[float], limit: int = 5) -> List[Dict[str, Any]]:
        """
        Searches for relevant chunks in the Qdrant collection given a query embedding.

        Args:
            query_embedding: The embedding vector of the query.
            limit: The maximum number of relevant chunks to retrieve.

        Returns:
            A list of dictionaries, where each dictionary represents a retrieved chunk.
        """
        print(f"Searching collection {COLLECTION_NAME} for relevant chunks...")
        search_result = await self.client.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_embedding,
            limit=limit,
            with_payload=True # Retrieve the full chunk payload
        )
        return [hit.payload for hit in search_result]

if __name__ == "__main__":
    async def main():
        db = QdrantVectorDB()
        # Ensure Qdrant is running via docker-compose before running this example

        # Example: Recreate collection
        await db.recreate_collection()

        # Example: Upsert some dummy data
        dummy_chunks = [
            {"chunk_id": "doc1-chunk1", "text": "The first paragraph about AI.", "metadata": {"title": "AI Basics", "url": "/docs/ai-basics", "nearest_heading": "Introduction"}},
            {"chunk_id": "doc1-chunk2", "text": "Advanced concepts in machine learning.", "metadata": {"title": "AI Basics", "url": "/docs/ai-basics", "nearest_heading": "Advanced ML"}},
            {"chunk_id": "doc2-chunk1", "text": "روبوٹکس کے بنیادی اصول۔", "metadata": {"title": "Robotika", "url": "/docs/robotics-urdu", "nearest_heading": "تعارف"}}
        ]
        dummy_embeddings = [
            [0.1] * VECTOR_SIZE, # Placeholder for actual embeddings
            [0.2] * VECTOR_SIZE,
            [0.3] * VECTOR_SIZE
        ]
        await db.upsert_chunks(dummy_chunks, dummy_embeddings)

        # Example: Search for a query
        query_text = "What is machine learning?"
        # In a real scenario, you'd get the embedding for query_text from Cohere
        query_embedding = [0.15] * VECTOR_SIZE # Placeholder

        print(f"\nSearching for: {query_text}")
        results = await db.search_chunks(query_embedding, limit=2)
        for i, res in enumerate(results):
            print(f"  Result {i+1}: {res['text']} (Source: {res['metadata']['document_url']}) - Heading: {res['metadata']['nearest_heading']}")

    asyncio.run(main())

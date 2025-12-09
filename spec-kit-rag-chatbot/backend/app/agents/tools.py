import os
import sys
from typing import List, Dict, Any
from openai_agents import tool

# Adjust the path to import from parent directories
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..'))

from app.vector.qdrant_client import QdrantVectorDB
from app.embedding.cohere_embedder import get_cohere_embeddings


qdrant_db = QdrantVectorDB()

@tool
async def retrieve_relevant_chunks(query: str) -> str:
    """
    Retrieves relevant document chunks from the vector database based on the user's query.
    This tool should be used when the user asks a question that requires knowledge
    from the indexed book content.

    Args:
        query (str): The user's question or query.

    Returns:
        str: A JSON string containing the retrieved document chunks and their metadata,
             formatted for the RAG agent to use.
    """
    print(f"Retrieving relevant chunks for query: {query}")
    try:
        query_embedding = await get_cohere_embeddings([query], model="embed-multilingual-v3.0")
        if not query_embedding:
            return "Error: Could not generate embedding for the query."

        search_results = await qdrant_db.search_chunks(query_embedding[0])

        if not search_results:
            return "No relevant information found in the knowledge base."

        formatted_results = []
        for hit in search_results:
            # Assuming payload contains 'text', 'metadata' (with 'document_title', 'document_url', 'nearest_heading')
            formatted_results.append({
                "chunk_text": hit.get("text", ""),
                "title": hit.get("metadata", {}).get("document_title", "N/A"),
                "url": hit.get("metadata", {}).get("document_url", "N/A"),
                "heading": hit.get("metadata", {}).get("nearest_heading", "N/A")
            })

        # Return as a JSON string for the agent to process
        import json
        return json.dumps(formatted_results, ensure_ascii=False)

    except Exception as e:
        return f"An error occurred during retrieval: {e}"

if __name__ == "__main__":
    async def main():
        # This example assumes Qdrant is running and populated with data
        # You would typically run app.ingestion.ingest first.

        # Example 1: Basic retrieval
        print("--- Example 1: Basic Retrieval ---")
        query_1 = "What are the basics of robotics?"
        results_1 = await retrieve_relevant_chunks(query_1)
        print(f"Results for '{query_1}':\n{results_1}\n")

        # Example 2: Roman Urdu query
        print("--- Example 2: Roman Urdu Retrieval ---")
        query_2 = "ReAct loop kya hota hai?"
        results_2 = await retrieve_relevant_chunks(query_2)
        print(f"Results for '{query_2}':\n{results_2}\n")

        # Example 3: Query with no expected results (for testing empty response)
        print("--- Example 3: No Results Expected ---")
        query_3 = "quantum entanglement in cooking"
        results_3 = await retrieve_relevant_chunks(query_3)
        print(f"Results for '{query_3}':\n{results_3}\n")

    asyncio.run(main())

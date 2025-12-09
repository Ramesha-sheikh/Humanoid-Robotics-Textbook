import os
from typing import List
import cohere
import asyncio
from dotenv import load_dotenv

load_dotenv()

COHERE_API_KEY = os.getenv("COHERE_API_KEY")
if not COHERE_API_KEY:
    raise ValueError("COHERE_API_KEY environment variable not set.")

co = cohere.AsyncClient(COHERE_API_KEY)

async def get_cohere_embeddings(texts: List[str], model: str = "embed-multilingual-v3.0") -> List[List[float]]:
    """
    Generates Cohere embeddings for a list of texts asynchronously.

    Args:
        texts: A list of strings to embed.
        model: The Cohere embedding model to use.

    Returns:
        A list of embedding vectors, where each vector is a list of floats.
    """
    try:
        response = await co.embed(
            texts=texts,
            model=model,
            input_type="search_document" # Or "search_query", "classification", "clustering" depending on use case
        )
        return response.embeddings
    except cohere.CohereAPIError as e:
        print(f"Cohere API error: {e}")
        raise
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        raise

if __name__ == "__main__":
    async def main():
        sample_texts = [
            "This is a test document about robotics.",
            "یہ ایک روبوٹکس کے بارے میں ٹیسٹ دستاویز ہے۔"
        ]
        try:
            embeddings = await get_cohere_embeddings(sample_texts)
            print("Embeddings generated successfully:")
            for i, emb in enumerate(embeddings):
                print(f"  Text {i+1} embedding (first 5 dims): {emb[:5]}...")
                print(f"  Embedding dimension: {len(emb)}")
        except Exception as e:
            print(f"Failed to get embeddings: {e}")

    asyncio.run(main())

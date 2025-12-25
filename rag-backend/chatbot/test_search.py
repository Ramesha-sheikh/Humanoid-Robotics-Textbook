from qdrant_client import QdrantClient
from cohere import Client as CohereClient
from config import config

# Initialize
qdrant = QdrantClient(url=config.QDRANT_URL, api_key=config.QDRANT_API_KEY)
cohere = CohereClient(config.COHERE_API_KEY)

print("Testing Cohere + Qdrant search...")
print(f"Collection: {config.COLLECTION_NAME}")

# Test query
question = "What is a robot?"
print(f"\nQuery: {question}")

try:
    # Create embedding
    print("Creating embedding with Cohere...")
    embedding = cohere.embed(
        texts=[question],
        model="embed-english-v3.0",
        input_type="search_query",
    ).embeddings[0]

    print(f"OK Embedding created: {len(embedding)} dimensions")

    # Search Qdrant
    print("\nSearching Qdrant...")
    results = qdrant.query_points(
        collection_name=config.COLLECTION_NAME,
        query=embedding,
        limit=5,
        score_threshold=0.3,  # Lower threshold
        with_payload=True,
    ).points

    print(f"OK Found {len(results)} results")

    if results:
        for i, r in enumerate(results):
            print(f"\n--- Result {i+1} (score: {r.score:.3f}) ---")
            print(f"URL: {r.payload.get('url', 'N/A')}")
            print(f"Text: {r.payload.get('chunk_text', '')[:100]}...")
    else:
        print("ERROR: No results found!")

        # Check collection info
        info = qdrant.get_collection(config.COLLECTION_NAME)
        print(f"\nCollection info:")
        print(f"  Points: {info.points_count}")
        print(f"  Vector config: {info.config}")

except Exception as e:
    print(f"ERROR: {e}")
    import traceback
    traceback.print_exc()

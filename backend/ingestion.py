from qdrant_client import QdrantClient, models

def initialize_qdrant(collection_name="my_collection"):
    client = QdrantClient(":memory:") # Use in-memory Qdrant for simplicity

    client.recreate_collection(
        collection_name=collection_name,
        vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),
    )
    return client

if __name__ == "__main__":
    client = initialize_qdrant()
    print(f"Qdrant client initialized with collection: {client.get_collections().collections[0].name}")

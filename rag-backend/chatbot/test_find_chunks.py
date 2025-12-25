from qdrant_client import QdrantClient
from cohere import Client as CohereClient
from config import config

# Initialize clients
qdrant = QdrantClient(url=config.QDRANT_URL, api_key=config.QDRANT_API_KEY)
cohere = CohereClient(config.COHERE_API_KEY)

# Test URL
page_url = "http://localhost:3000/docs/introduction"
page_path = "https://humanoid-robotics-textbook-psi.vercel.app/docs/introduction/"

print(f"Looking for: {page_path}")

# Extract search query
page_name = page_path.split('/')[-2] if page_path.endswith('/') else page_path.split('/')[-1]
search_query = page_name.replace('-', ' ')
print(f"Search query: {search_query}")

# Create embedding
print("Creating embedding...")
query_embedding = cohere.embed(
    texts=[search_query],
    model="embed-english-v3.0",
    input_type="search_query",
).embeddings[0]

print("Searching Qdrant...")
search_results = qdrant.query_points(
    collection_name=config.COLLECTION_NAME,
    query=query_embedding,
    limit=50,
    with_payload=True,
).points

print(f"Found {len(search_results)} results from search")

# Filter exact matches
matching = [p for p in search_results if p.payload.get("url", "") == page_path]
print(f"Exact URL matches: {len(matching)}")

if matching:
    print(f"\nFirst match:")
    print(f"  URL: {matching[0].payload.get('url')}")
    print(f"  Text preview: {matching[0].payload.get('chunk_text', '')[:150]}...")
else:
    print("\n All URLs found:")
    for p in search_results[:5]:
        print(f"  - {p.payload.get('url')}")

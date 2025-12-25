from qdrant_client import QdrantClient
from config import config

client = QdrantClient(url=config.QDRANT_URL, api_key=config.QDRANT_API_KEY)

# Get collection info
info = client.get_collection('book')
print(f'Points count: {info.points_count}')

# Get sample data
result = client.scroll('book', limit=3, with_payload=True)
if result[0]:
    for i, point in enumerate(result[0]):
        print(f'\n=== Point {i+1} ===')
        print(f'ID: {point.id}')
        print(f'Payload keys: {list(point.payload.keys())}')
        if 'url' in point.payload:
            print(f'URL: {point.payload["url"]}')
        if 'chunk_text' in point.payload:
            print(f'Text preview: {point.payload["chunk_text"][:100]}...')

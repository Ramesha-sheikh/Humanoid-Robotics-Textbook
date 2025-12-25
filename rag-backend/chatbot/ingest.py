import asyncio
import logging
import uuid
from datetime import datetime
from xml.etree import ElementTree as ET
import requests
from bs4 import BeautifulSoup
from cohere import Client as CohereClient
from qdrant_client import QdrantClient
from qdrant_client.http.models import PointStruct, VectorParams, Distance
from trafilatura import extract
# langchain wala import hata diya
from config import config
from utils import clean_text

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("ingest")

cohere_client = CohereClient(config.COHERE_API_KEY)
qdrant_client = QdrantClient(url=config.QDRANT_URL, api_key=config.QDRANT_API_KEY)

def create_collection():
    try:
        qdrant_client.get_collection(config.COLLECTION_NAME)
        logger.info("Collection already exists")
    except:
        qdrant_client.create_collection(
            collection_name=config.COLLECTION_NAME,
            vectors_config={
                config.VECTOR_NAME: VectorParams(size=1024, distance=Distance.COSINE)
            }
        )
        logger.info("Collection created")

def get_urls_from_sitemap():
    headers = {"User-Agent": "Mozilla/5.0"}
    resp = requests.get(config.SITEMAP_URL, headers=headers, timeout=30)
    resp.raise_for_status()
    root = ET.fromstring(resp.content)
    ns = "{http://www.sitemaps.org/schemas/sitemap/0.9}"
    urls = [loc.text.strip() for url in root.findall(f'.//{ns}url') for loc in url.findall(f'{ns}loc')]
    logger.info(f"Found {len(urls)} URLs")
    return urls

async def extract_content(url: str):
    headers = {"User-Agent": "Mozilla/5.0"}
    try:
        resp = requests.get(url, headers=headers, timeout=30)
        resp.raise_for_status()
        text = extract(resp.text, favor_precision=True)
        if text and len(text.strip()) > 150:
            title = BeautifulSoup(resp.text, "html.parser").title.string.strip() if BeautifulSoup(resp.text, "html.parser").title else url.split("/")[-1]
            return {"url": url, "title": title, "text": clean_text(text)}
    except Exception as e:
        logger.warning(f"Failed {url}: {e}")
    return None

async def main():
    create_collection()
    urls = get_urls_from_sitemap()
    pages = []
    for url in urls:
        doc = await extract_content(url)
        if doc:
            pages.append(doc)
        await asyncio.sleep(1)

    logger.info(f"Extracted {len(pages)} pages")

    # Simple character-based chunking (tiktoken ke baghair)
    CHUNK_SIZE = 1400    # characters
    CHUNK_OVERLAP = 200

    points = []
    total_chunks = 0

    for page in pages:
        text = page["text"]
        chunks = []
        start = 0
        while start < len(text):
            end = start + CHUNK_SIZE
            chunk = text[start:end]
            if chunk.strip():
                chunks.append(chunk.strip())
            start = end - CHUNK_OVERLAP  # overlap

        if not chunks:
            continue

        embeddings = cohere_client.embed(
            texts=chunks,
            model=config.COHERE_MODEL,
            input_type="search_document"
        ).embeddings

        for i, (chunk, vec) in enumerate(zip(chunks, embeddings)):
            points.append(PointStruct(
                id=str(uuid.uuid4()),
                vector=vec,
                payload={
                    "chunk_text": chunk,
                    "url": page["url"],
                    "title": page["title"],
                    "chunk_index": i,
                    "timestamp": datetime.utcnow().isoformat()
                }
            ))
            total_chunks += 1

        if len(points) >= config.BATCH_SIZE:
            qdrant_client.upsert(collection_name=config.COLLECTION_NAME, points=points)
            logger.info(f"Uploaded {len(points)} chunks")
            points = []

    if points:
        qdrant_client.upsert(collection_name=config.COLLECTION_NAME, points=points)

    logger.info(f"Ingestion complete! Total chunks: {total_chunks} 🚀")

if __name__ == "__main__":
    asyncio.run(main())
from dotenv import load_dotenv
import os

load_dotenv()

class Config:
    COHERE_API_KEY = os.getenv("COHERE_API_KEY")
    QDRANT_URL = os.getenv("QDRANT_URL")
    QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
    
    # Tumhari book ka sitemap
    SITEMAP_URL = os.getenv("SITEMAP_URL", "https://humanoid-robotics-textbook-psi.vercel.app/sitemap.xml")
    
    COLLECTION_NAME = "book"                    # Tumhara collection
    VECTOR_NAME = "content"
    COHERE_MODEL = "embed-english-v3.0"
    
    CHUNK_SIZE = 400
    CHUNK_OVERLAP = 60
    BATCH_SIZE = 64

config = Config()
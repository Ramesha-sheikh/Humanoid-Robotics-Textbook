import os
from pydantic_settings import BaseSettings, SettingsConfigDict

class Settings(BaseSettings):
    model_config = SettingsConfigDict(env_file=".env", extra="ignore")

    COHERE_API_KEY: str
    GROQ_API_KEY: str | None = None  # Optional, if using OpenAI as fallback
    OPENAI_API_KEY: str | None = None # Optional, if using Groq as primary

    QDRANT_HOST: str = "localhost"
    QDRANT_PORT: int = 6333

    BACKEND_CORS_ORIGINS: str = "*" # Comma-separated URLs, e.g., "http://localhost:3000,https://your-docusaurus-site.com"


settings = Settings()
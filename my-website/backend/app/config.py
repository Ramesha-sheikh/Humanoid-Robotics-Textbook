"""
Configuration management using Pydantic Settings.
Loads environment variables from .env file.
"""

from pydantic_settings import BaseSettings
from typing import List


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # Gemini API
    gemini_api_key: str

    # Qdrant Cloud
    qdrant_url: str
    qdrant_api_key: str

    # Neon Postgres
    neon_database_url: str

    # CORS
    cors_origins: List[str] = ["http://localhost:3000"]

    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"

        @classmethod
        def parse_env_var(cls, field_name: str, raw_val: str):
            # Parse comma-separated CORS origins
            if field_name == "cors_origins":
                return [origin.strip() for origin in raw_val.split(",")]
            return raw_val


# Global settings instance
settings = Settings()


# Validate on import
def validate_settings():
    """Validate that all required settings are present."""
    required_fields = [
        "gemini_api_key",
        "qdrant_url",
        "qdrant_api_key",
        "neon_database_url"
    ]

    missing = []
    for field in required_fields:
        if not getattr(settings, field, None):
            missing.append(field.upper())

    if missing:
        raise ValueError(
            f"Missing required environment variables: {', '.join(missing)}\n"
            f"Please create a .env file based on .env.example"
        )


# Validate on import (will fail fast if config is invalid)
try:
    validate_settings()
    print("✅ Configuration loaded successfully")
except ValueError as e:
    print(f"⚠️  Configuration warning: {e}")

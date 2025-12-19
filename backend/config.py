
from pydantic_settings import BaseSettings
from typing import Optional

class Settings(BaseSettings):
    groq_api_key: str
    qdrant_api_key: str
    qdrant_url: str
    database_url: str
    gemini_api_key: Optional[str] = None  # Optional, not used anymore

    class Config:
        env_file = ".env"

settings = Settings()

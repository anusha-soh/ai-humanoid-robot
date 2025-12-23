from pydantic_settings import BaseSettings, SettingsConfigDict

class Settings(BaseSettings):
    model_config = SettingsConfigDict(env_file=".env", case_sensitive=False)

    # Gemini
    gemini_api_key: str
    gemini_model: str = "gemini-2.0-flash-exp"
    gemini_embedding_model: str = "text-embedding-004"

    # Qdrant
    qdrant_url: str
    qdrant_api_key: str
    qdrant_collection: str = "docusaurus-book"

    # Neon Postgres
    database_url: str

    # CORS
    allowed_origins: str = "http://localhost:3000"

    # App
    log_level: str = "INFO"

    @property
    def allowed_origins_list(self) -> list[str]:
        return [origin.strip() for origin in self.allowed_origins.split(",")]

settings = Settings()

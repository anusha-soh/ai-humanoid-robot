"""
Quick verification script to test .env configuration
Run: python test_config.py
"""
import sys
from pathlib import Path

# Add app to path
sys.path.insert(0, str(Path(__file__).parent))

try:
    from app.core.config import settings

    print("[OK] Configuration loaded successfully!\n")
    print("=" * 50)
    print("CONFIGURATION CHECK")
    print("=" * 50)

    # Check Gemini
    print(f"\nGemini API:")
    print(f"   Model: {settings.gemini_model}")
    print(f"   Embedding Model: {settings.gemini_embedding_model}")
    print(f"   API Key: {'[OK] SET' if settings.gemini_api_key else '[MISSING]'}")

    # Check Qdrant
    print(f"\nQdrant:")
    print(f"   URL: {settings.qdrant_url}")
    print(f"   Collection: {settings.qdrant_collection}")
    print(f"   API Key: {'[OK] SET' if settings.qdrant_api_key else '[MISSING]'}")

    # Check Database
    print(f"\nDatabase:")
    print(f"   URL: {settings.database_url[:30]}... {'[OK] SET' if settings.database_url else '[MISSING]'}")

    # Check CORS
    print(f"\nCORS:")
    print(f"   Allowed Origins: {settings.allowed_origins_list}")

    print(f"\n{'=' * 50}")
    print("[OK] All configurations loaded successfully!")
    print("You're ready to proceed with Phase 3!\n")

except Exception as e:
    print(f"[ERROR] Configuration Error: {e}")
    print("\nMake sure:")
    print("   1. backend/.env file exists")
    print("   2. All placeholders are replaced with real values")
    print("   3. No syntax errors in .env file")
    sys.exit(1)

#!/usr/bin/env python3
"""
Simple test to verify that the backend modules can be imported without errors
"""

def test_imports():
    print("Testing backend module imports...")

    try:
        from backend.src.api.main import app
        print("✓ Successfully imported main FastAPI app")
    except Exception as e:
        print(f"✗ Error importing main app: {e}")
        return False

    try:
        from backend.src.api import user_routes
        print("✓ Successfully imported user_routes")
    except Exception as e:
        print(f"✗ Error importing user_routes: {e}")
        return False

    try:
        from backend.src.services.user_service import UserService
        print("✓ Successfully imported UserService")
    except Exception as e:
        print(f"✗ Error importing UserService: {e}")
        return False

    try:
        from backend.src.lib.database import engine, Base
        print("✓ Successfully imported database components")
    except Exception as e:
        print(f"✗ Error importing database components: {e}")
        return False

    try:
        from backend.src.models.user import User, UserCreate, UserResponse
        print("✓ Successfully imported User models")
    except Exception as e:
        print(f"✗ Error importing User models: {e}")
        return False

    try:
        from backend.src.models.chapter import Chapter, ChapterResponse
        print("✓ Successfully imported Chapter models")
    except Exception as e:
        print(f"✗ Error importing Chapter models: {e}")
        return False

    try:
        from backend.src.models.learning_path import LearningPath
        print("✓ Successfully imported LearningPath models")
    except Exception as e:
        print(f"✗ Error importing LearningPath models: {e}")
        return False

    try:
        from backend.src.models.ai_interaction import AIChatSession
        print("✓ Successfully imported AIInteraction models")
    except Exception as e:
        print(f"✗ Error importing AIInteraction models: {e}")
        return False

    try:
        from backend.src.models.citation import Citation
        print("✓ Successfully imported Citation models")
    except Exception as e:
        print(f"✗ Error importing Citation models: {e}")
        return False

    try:
        from backend.src.lib.vector_store import vector_store
        print("✓ Successfully imported vector store")
    except Exception as e:
        print(f"✗ Error importing vector store: {e}")
        return False

    print("\n✓ All imports successful! Backend structure is correct.")
    return True

if __name__ == "__main__":
    test_imports()
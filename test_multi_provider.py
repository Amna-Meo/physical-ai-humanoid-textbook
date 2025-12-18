#!/usr/bin/env python3
"""
Test script to demonstrate the multi-AI provider functionality
"""

import os
import sys
from unittest.mock import patch, MagicMock

# Add the backend src directory to the path
sys.path.append(os.path.join(os.path.dirname(__file__), 'backend', 'src'))

def test_openai_provider():
    """Test the OpenAI provider configuration"""
    print("Testing OpenAI provider configuration...")

    # Mock the OpenAI import and client
    with patch.dict('sys.modules', {
        'openai': MagicMock(),
        'openai.OpenAI': MagicMock(),
    }):
        with patch('os.getenv', side_effect=lambda x, default=None: 'test-key' if x == 'OPENAI_API_KEY' else default):
            # Reload the module to pick up the mocked imports
            import importlib
            import backend.src.services.ai_service
            importlib.reload(backend.src.services.ai_service)

            from backend.src.services.ai_service import AI_PROVIDER, OPENAI_AVAILABLE
            print(f"  AI Provider: {AI_PROVIDER}")
            print(f"  OpenAI Available: {OPENAI_AVAILABLE}")

            if AI_PROVIDER == "openai" and OPENAI_AVAILABLE:
                print("  ✅ OpenAI provider configured correctly")
            else:
                print("  ❌ OpenAI provider not configured correctly")

def test_gemini_provider():
    """Test the Gemini provider configuration"""
    print("\nTesting Gemini provider configuration...")

    # Mock the google.generativeai import
    with patch.dict('sys.modules', {
        'google.generativeai': MagicMock(),
        'google': MagicMock(),
    }):
        with patch('os.getenv', side_effect=lambda x, default=None: 'test-key' if x == 'GEMINI_API_KEY' else default):
            # Reload the module to pick up the mocked imports
            import importlib
            import backend.src.services.ai_service
            importlib.reload(backend.src.services.ai_service)

            from backend.src.services.ai_service import AI_PROVIDER, GEMINI_AVAILABLE
            print(f"  AI Provider: {AI_PROVIDER}")
            print(f"  Gemini Available: {GEMINI_AVAILABLE}")

            if AI_PROVIDER == "gemini" and GEMINI_AVAILABLE:
                print("  ✅ Gemini provider configured correctly")
            else:
                print("  ❌ Gemini provider not configured correctly")

def test_mock_provider():
    """Test the mock provider fallback"""
    print("\nTesting mock provider fallback...")

    # Make sure no AI providers are available
    with patch('os.getenv', return_value=None):
        # Reload the module to pick up the changes
        import importlib
        import backend.src.services.ai_service
        importlib.reload(backend.src.services.ai_service)

        from backend.src.services.ai_service import AI_PROVIDER, OPENAI_AVAILABLE, GEMINI_AVAILABLE
        print(f"  AI Provider: {AI_PROVIDER}")
        print(f"  OpenAI Available: {OPENAI_AVAILABLE}")
        print(f"  Gemini Available: {GEMINI_AVAILABLE}")

        if AI_PROVIDER == "mock":
            print("  ✅ Mock provider fallback working correctly")
        else:
            print("  ❌ Mock provider fallback not working correctly")

if __name__ == "__main__":
    print("Testing Multi-AI Provider Configuration")
    print("=" * 50)

    test_openai_provider()
    test_gemini_provider()
    test_mock_provider()

    print("\n" + "=" * 50)
    print("Multi-AI provider system test completed!")
    print("The system can now use OpenAI, Google Gemini, or mock responses based on availability.")
    print("\nTo use a specific provider, set the corresponding environment variable:")
    print("  - OPENAI_API_KEY for OpenAI")
    print("  - GEMINI_API_KEY for Google Gemini")
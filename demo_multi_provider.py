#!/usr/bin/env python3
"""
Demonstration of the multi-AI provider functionality
"""

import os

def demo_provider_selection():
    """Demonstrate how the system selects AI providers"""
    print("Multi-AI Provider System Demonstration")
    print("=" * 50)

    # Simulate the provider selection logic
    print("\n1. Provider Selection Logic:")
    print("   The system checks for available AI providers in this order:")
    print("   - OpenAI (if OPENAI_API_KEY is set)")
    print("   - Google Gemini (if GEMINI_API_KEY is set)")
    print("   - Mock responses (fallback)")

    print("\n2. Configuration Examples:")
    print("\n   For OpenAI:")
    print("   export OPENAI_API_KEY='your-openai-api-key'")
    print("   export OPENAI_MODEL='gpt-3.5-turbo'  # or 'gpt-4'")

    print("\n   For Google Gemini:")
    print("   export GEMINI_API_KEY='your-gemini-api-key'")
    print("   (The system will use gemini-pro model)")

    print("\n   No API key (mock responses):")
    print("   Simply don't set either environment variable")

    # Show current configuration based on environment
    print("\n3. Current Configuration:")
    openai_key = os.getenv("OPENAI_API_KEY")
    gemini_key = os.getenv("GEMINI_API_KEY")

    if openai_key:
        print(f"   ✓ OPENAI_API_KEY is set (first {min(len(openai_key), 8)} characters: {openai_key[:8]}...)")
        print("   → System will use OpenAI provider")
    elif gemini_key:
        print(f"   ✓ GEMINI_API_KEY is set (first {min(len(gemini_key), 8)} characters: {gemini_key[:8]}...)")
        print("   → System will use Google Gemini provider")
    else:
        print("   - No API keys are set")
        print("   → System will use mock responses (great for testing!)")

    print("\n4. Benefits:")
    print("   ✓ Use free Google Gemini API instead of paid OpenAI")
    print("   ✓ Fallback to mock responses when no API is available")
    print("   ✓ Easy switching between providers")
    print("   ✓ Same interface regardless of provider used")

    print("\n5. Try it out:")
    print("   Set your preferred API key environment variable and run the application!")
    print("   The AI service will automatically use the best available provider.")

if __name__ == "__main__":
    demo_provider_selection()
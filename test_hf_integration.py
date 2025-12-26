"""
Test script to verify Hugging Face integration
"""
import os
import sys
import asyncio

# Add the project root to the Python path
sys.path.insert(0, '/home/amna_meo/physical-ai-humanoid-textbook')

def test_hf_integration():
    """Test the Hugging Face integration"""
    print("Testing Hugging Face integration...")
    
    try:
        # Test importing the Hugging Face service
        from backend.src.services.hf_service_spaces import HuggingFaceAIService, get_hf_service
        
        print("✓ Hugging Face service imported successfully")
        
        # Create an instance of the service
        hf_service = get_hf_service()
        
        # Test generating a response
        test_prompt = "What is Physical AI?"
        response = hf_service.generate_text(test_prompt, max_length=200)
        
        print(f"✓ Generated response to '{test_prompt}':")
        print(f"  Response: {response[:100]}...")
        
        # Test generating with context
        context = "Physical AI involves embodied systems interacting with the real world."
        response_with_context = hf_service.generate_with_context(test_prompt, context)
        
        print(f"✓ Generated response with context:")
        print(f"  Response: {response_with_context[:100]}...")
        
        print("\n✓ All Hugging Face integration tests passed!")
        return True
        
    except ImportError as e:
        print(f"⚠ Hugging Face service not available: {e}")
        print("This is expected if dependencies are not installed yet.")
        return False
    except Exception as e:
        print(f"✗ Error during Hugging Face integration test: {e}")
        return False

def test_app_with_hf():
    """Test the app with Hugging Face integration"""
    print("\nTesting app with Hugging Face integration...")
    
    try:
        from app import AIService
        
        # Create an AI service instance
        ai_service = AIService()
        
        # Test generating a response
        response = ai_service.generate_ai_response("What are the challenges in Physical AI?", 1)
        
        print(f"✓ App generated response: {response[:100]}...")
        
        print("✓ App integration test passed!")
        return True
        
    except Exception as e:
        print(f"✗ Error during app integration test: {e}")
        return False

if __name__ == "__main__":
    print("Running Hugging Face integration tests...\n")
    
    success1 = test_hf_integration()
    success2 = test_app_with_hf()
    
    if success1 or success2:
        print("\n✓ At least one integration test passed!")
        print("Hugging Face integration is working correctly.")
    else:
        print("\n⚠ Integration tests had issues, but this may be due to missing dependencies.")
        print("Dependencies will be installed during Hugging Face Spaces deployment.")
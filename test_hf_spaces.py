"""
Test script for Hugging Face Spaces deployment
This script tests the basic functionality of the deployed backend
"""
import requests
import time
import sys

def test_hf_spaces_backend():
    """Test the Hugging Face Spaces backend deployment"""
    print("Testing Hugging Face Spaces backend deployment...")
    
    # Use the local server for testing
    base_url = "http://localhost:7860"
    
    print(f"Testing endpoints at {base_url}")
    
    # Test health endpoint
    try:
        response = requests.get(f"{base_url}/health")
        if response.status_code == 200:
            health_data = response.json()
            print(f"✓ Health check passed: {health_data}")
        else:
            print(f"✗ Health check failed with status {response.status_code}")
            return False
    except Exception as e:
        print(f"✗ Health check error: {str(e)}")
        return False
    
    # Test root endpoint
    try:
        response = requests.get(f"{base_url}/")
        if response.status_code == 200:
            root_data = response.json()
            print(f"✓ Root endpoint accessible: {root_data.get('message', 'No message')}")
        else:
            print(f"✗ Root endpoint failed with status {response.status_code}")
            return False
    except Exception as e:
        print(f"✗ Root endpoint error: {str(e)}")
        return False
    
    # Test creating a chat session
    try:
        response = requests.post(f"{base_url}/api/v1/ai/chat-sessions", 
                                json={"session_title": "Test Session"})
        if response.status_code == 200:
            session_data = response.json()
            session_id = session_data["id"]
            print(f"✓ Created chat session with ID: {session_id}")
        else:
            print(f"✗ Failed to create chat session with status {response.status_code}")
            print(f"Response: {response.text}")
            return False
    except Exception as e:
        print(f"✗ Chat session creation error: {str(e)}")
        return False
    
    # Test sending a message
    try:
        test_message = {"role": "user", "content": "What is Physical AI?"}
        response = requests.post(f"{base_url}/api/v1/ai/chat-sessions/{session_id}/messages", 
                                json=test_message)
        if response.status_code == 200:
            message_response = response.json()
            print(f"✓ Message sent successfully, AI response received")
            print(f"  Query: {test_message['content']}")
            print(f"  Response: {message_response['content'][:100]}...")
        else:
            print(f"✗ Failed to send message with status {response.status_code}")
            print(f"Response: {response.text}")
            return False
    except Exception as e:
        print(f"✗ Message sending error: {str(e)}")
        return False
    
    # Test getting session messages
    try:
        response = requests.get(f"{base_url}/api/v1/ai/chat-sessions/{session_id}/messages")
        if response.status_code == 200:
            messages = response.json()
            print(f"✓ Retrieved {len(messages)} messages from session")
        else:
            print(f"✗ Failed to get session messages with status {response.status_code}")
            print(f"Response: {response.text}")
            return False
    except Exception as e:
        print(f"✗ Session messages retrieval error: {str(e)}")
        return False
    
    print("\n✓ All tests passed! Hugging Face Spaces backend is working correctly.")
    return True

if __name__ == "__main__":
    success = test_hf_spaces_backend()
    if not success:
        print("\n✗ Some tests failed. Please check the deployment.")
        sys.exit(1)
    else:
        print("\n✓ All tests passed successfully!")
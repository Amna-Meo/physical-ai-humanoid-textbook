"""
Gradio interface for Physical AI & Humanoid Robotics Textbook
This provides a user-friendly UI for the Hugging Face Space
"""
import gradio as gr
import requests
import json
import os
import uuid

# Use the backend API
BASE_URL = os.getenv("BASE_URL", "http://localhost:8000")

# Session storage (in a real implementation, this would be persistent)
sessions = {}

def create_session():
    """Create a new chat session"""
    try:
        response = requests.post(f"{BASE_URL}/api/v1/ai/chat-sessions", 
                                json={"session_title": f"Session {str(uuid.uuid4())[:8]}"})
        if response.status_code == 200:
            session_data = response.json()
            session_id = session_data["id"]
            sessions[session_id] = []
            return session_id, f"Created new session: {session_data['session_title']}"
        else:
            return None, f"Error creating session: {response.status_code}"
    except Exception as e:
        return None, f"Error creating session: {str(e)}"

def chat_with_ai(message, history, session_id):
    """Chat with the AI assistant"""
    if not session_id:
        session_id, msg = create_session()
        if not session_id:
            return msg, history, session_id
    
    try:
        # Send message to backend
        response = requests.post(
            f"{BASE_URL}/api/v1/ai/chat-sessions/{session_id}/messages",
            json={"role": "user", "content": message}
        )
        
        if response.status_code == 200:
            response_data = response.json()
            ai_response = response_data["content"]
            
            # Update history
            history = history + [(message, ai_response)]
            return "", history, session_id
        else:
            return f"Error: {response.status_code}", history, session_id
    except Exception as e:
        return f"Error: {str(e)}", history, session_id

def reset_chat():
    """Reset the chat"""
    return [], None, "New session will be created on first message"

# Create Gradio interface
with gr.Blocks(title="Physical AI & Humanoid Robotics Textbook") as demo:
    gr.Markdown("""
    # Physical AI & Humanoid Robotics Textbook
    
    This is an interactive AI assistant for learning about Physical AI and Humanoid Robotics. 
    Ask questions about topics like:
    - Physical AI principles
    - Humanoid robotics
    - ROS 2 for humanoid robotics
    - Gazebo simulation
    - NVIDIA Isaac Sim
    - Perception systems
    - Planning and navigation
    """)
    
    session_id_state = gr.State(value=None)
    
    with gr.Row():
        with gr.Column(scale=1):
            new_session_btn = gr.Button("Start New Session")
        with gr.Column(scale=4):
            session_info = gr.Textbox(label="Session Info", interactive=False)
    
    with gr.Row():
        chatbot = gr.Chatbot(label="AI Assistant").style(height=500)
    
    with gr.Row():
        msg = gr.Textbox(label="Your Message", placeholder="Ask a question about Physical AI or Humanoid Robotics...")
        submit_btn = gr.Button("Send")
    
    with gr.Row():
        reset_btn = gr.Button("Reset Chat")
    
    # Event handling
    submit_btn.click(
        chat_with_ai,
        inputs=[msg, chatbot, session_id_state],
        outputs=[msg, chatbot, session_id_state]
    )
    
    msg.submit(
        chat_with_ai,
        inputs=[msg, chatbot, session_id_state],
        outputs=[msg, chatbot, session_id_state]
    )
    
    new_session_btn.click(
        create_session,
        inputs=[],
        outputs=[session_id_state, session_info]
    )
    
    reset_btn.click(
        reset_chat,
        inputs=[],
        outputs=[chatbot, session_id_state, session_info]
    )

# Launch the interface
if __name__ == "__main__":
    demo.launch(server_name="0.0.0.0", server_port=int(os.getenv("PORT", 7860)))
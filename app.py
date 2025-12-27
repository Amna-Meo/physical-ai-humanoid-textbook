"""
Hugging Face Space app for Physical AI & Humanoid Robotics Textbook
This app provides a Gradio interface for the AI textbook backend
"""
import os
import sys
from contextlib import contextmanager
from typing import Optional, List, Dict, Any
import logging
from datetime import datetime
from fastapi import FastAPI, Request, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from pydantic import BaseModel
from dataclasses import dataclass, field
import hashlib
import gradio as gr

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Import Hugging Face service
try:
    from backend.src.services.hf_service_spaces import get_hf_service, HuggingFaceAIService
    HF_AVAILABLE = True
    hf_service_instance = get_hf_service()
except ImportError:
    logger.info("Hugging Face service not available, using mock responses")
    HF_AVAILABLE = False
    hf_service_instance = None

# Mock database implementation using in-memory storage
@dataclass
class MockDB:
    """In-memory database for Hugging Face Spaces"""
    chat_sessions: Dict[int, dict] = field(default_factory=dict)
    chat_messages: Dict[int, dict] = field(default_factory=dict)
    interaction_logs: Dict[int, dict] = field(default_factory=dict)
    next_session_id: int = 1
    next_message_id: int = 1
    next_log_id: int = 1

    def create_chat_session(self, session_data: dict) -> dict:
        session_id = self.next_session_id
        self.next_session_id += 1
        session = {
            "id": session_id,
            "user_id": session_data.get("user_id"),
            "session_title": session_data.get("session_title", f"Session {session_id}"),
            "is_active": True,
            "session_metadata": session_data.get("session_metadata", {}),
            "created_at": datetime.utcnow().isoformat(),
            "updated_at": datetime.utcnow().isoformat()
        }
        self.chat_sessions[session_id] = session
        return session

    def get_chat_session(self, session_id: int) -> Optional[dict]:
        return self.chat_sessions.get(session_id)

    def add_message(self, message_data: dict) -> dict:
        message_id = self.next_message_id
        self.next_message_id += 1
        message = {
            "id": message_id,
            "session_id": message_data.get("session_id"),
            "user_id": message_data.get("user_id"),
            "message_type": message_data.get("message_type"),
            "content": message_data.get("content"),
            "role": message_data.get("role"),
            "sources": message_data.get("sources", []),
            "context_chapters": message_data.get("context_chapters", []),
            "timestamp": datetime.utcnow().isoformat()
        }
        self.chat_messages[message_id] = message
        return message

    def get_session_messages(self, session_id: int) -> List[dict]:
        messages = [msg for msg in self.chat_messages.values() if msg["session_id"] == session_id]
        return sorted(messages, key=lambda x: x["timestamp"])

    def log_interaction(self, log_data: dict) -> dict:
        log_id = self.next_log_id
        self.next_log_id += 1
        log_entry = {
            "id": log_id,
            "user_id": log_data.get("user_id"),
            "session_id": log_data.get("session_id"),
            "interaction_type": log_data.get("interaction_type", "chat"),
            "input_text": log_data.get("input_text"),
            "output_text": log_data.get("output_text"),
            "ai_model_used": log_data.get("ai_model_used", "mock-model"),
            "created_at": datetime.utcnow().isoformat()
        }
        self.interaction_logs[log_id] = log_entry
        return log_entry

# Global mock database instance
mock_db = MockDB()

# AI service with Hugging Face integration
class AIService:
    def __init__(self):
        self.model_used = "huggingface-model" if HF_AVAILABLE else "mock-model"
        
    def create_chat_session(self, user_id: Optional[int] = None, session_title: Optional[str] = None) -> dict:
        session_data = {
            "user_id": user_id,
            "session_title": session_title
        }
        return mock_db.create_chat_session(session_data)

    def get_chat_session(self, session_id: int) -> Optional[dict]:
        return mock_db.get_chat_session(session_id)

    def add_message_to_session(self, session_id: int, user_id: Optional[int], role: str, content: str) -> dict:
        message_data = {
            "session_id": session_id,
            "user_id": user_id,
            "message_type": role,
            "role": role,
            "content": content
        }
        return mock_db.add_message(message_data)

    def generate_ai_response(self, query: str, session_id: int, user_id: Optional[int] = None,
                           context_chapters: Optional[List[int]] = None) -> str:
        """Generate an AI response based on the query, using Hugging Face if available"""
        if HF_AVAILABLE and hf_service_instance:
            # Use Hugging Face model for response
            context_text = "No specific textbook content found for this query. Use general knowledge about Physical AI and Humanoid Robotics."
            response = hf_service_instance.generate_with_context(query, context_text)
            return response
        else:
            # Fallback to mock responses
            return self._generate_mock_response(query)

    def _generate_mock_response(self, query: str) -> str:
        """Generate a mock AI response based on the query"""
        query_lower = query.lower()

        if 'physical ai' in query_lower or 'embodiment' in query_lower:
            return """Physical AI represents a paradigm shift where cognitive systems are embodied in physical form and interact with the real world.
            Unlike traditional AI that operates in virtual environments, Physical AI systems must navigate the complexities of physics, dynamics, and real-world uncertainty.
            The core principles include:
            1. Embodiment: Intelligence emerges from the interaction between an agent and its physical environment
            2. Real-time Processing: Systems must respond to environmental changes within strict temporal constraints
            3. Uncertainty Management: Dealing with sensor noise, actuator limitations, and environmental variability
            4. Energy Efficiency: Optimizing for real-world power constraints"""

        elif 'humanoid' in query_lower or 'robot' in query_lower:
            return """Humanoid robotics is a key application area for Physical AI. Humanoid robots must perform complex tasks in human environments,
            requiring sophisticated locomotion, manipulation, and interaction capabilities. Key applications include:
            - Locomotion: Walking, running, and navigating complex terrains
            - Manipulation: Grasping objects with human-like dexterity
            - Interaction: Communicating and collaborating with humans
            - Adaptation: Learning from physical interactions to improve performance"""

        elif 'challenge' in query_lower or 'difficulty' in query_lower:
            return """The main technical challenges in Physical AI include:
            1. Sim-to-Real Gap: Bridging the difference between simulation and real-world performance
            2. Safety: Ensuring safe operation around humans and environments
            3. Scalability: Developing systems that can operate reliably across diverse scenarios
            4. Learning Efficiency: Acquiring new skills with minimal physical interaction"""

        elif 'ros' in query_lower or 'ros2' in query_lower:
            return """ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. 
            It provides libraries and tools to help software developers create robot applications. 
            Key features include:
            - Distributed computing architecture
            - Support for multiple programming languages
            - Hardware abstraction
            - Device drivers
            - Libraries for implementing commonly used functionality"""

        elif 'gazebo' in query_lower or 'simulation' in query_lower:
            return """Gazebo is a 3D simulation environment for autonomous robots. It provides:
            - High-fidelity physics simulation
            - Advanced 3D graphics
            - A library of robot models and environments
            - Integration with ROS/ROS2
            - Multiple sensors with realistic behavior
            - Dynamic interaction between objects"""

        else:
            return f"""Based on the textbook content: Physical AI and Humanoid Robotics involve complex interactions between cognitive systems and physical environments.
            The field addresses challenges like real-time processing, uncertainty management, and energy efficiency.
            In response to your query about "{query[:50]}{'...' if len(query) > 50 else ''}", these fundamental concepts are essential for understanding embodied AI systems."""

    def get_session_history(self, session_id: int, limit: int = 50) -> List[dict]:
        """Get the message history for a session"""
        return mock_db.get_session_messages(session_id)[-limit:]

# Global AI service instance
ai_service = AIService()

# Create the FastAPI app
app = FastAPI(
    title="Physical AI & Humanoid Robotics Textbook API",
    description="Backend API for the Physical AI & Humanoid Robotics Textbook platform with Hugging Face AI integration",
    version="0.1.0"
)

# Add CORS middleware for Hugging Face Spaces
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allow_headers=["*"],
)

# Pydantic models for API requests/responses
class AIChatSessionCreate(BaseModel):
    session_title: Optional[str] = None

class AIChatSessionResponse(BaseModel):
    id: int
    user_id: Optional[int] = None
    session_title: Optional[str] = None
    is_active: bool
    session_metadata: Optional[dict] = None
    created_at: str
    updated_at: Optional[str] = None

class AIChatMessageCreate(BaseModel):
    role: str
    content: str
    context_chapters: Optional[List[int]] = None

class AIChatMessageResponse(BaseModel):
    id: int
    session_id: int
    user_id: Optional[int] = None
    message_type: str
    content: str
    role: str
    sources: Optional[List[dict]] = None
    context_chapters: Optional[List[int]] = None
    timestamp: str
    is_grounding_validated: bool = False

# Routes
@app.get("/")
async def root():
    return {
        "message": "Welcome to the Physical AI & Humanoid Robotics Textbook API",
        "description": "This is a Hugging Face Spaces deployment of the textbook backend with Hugging Face AI integration",
        "endpoints": [
            "/health",
            "/api/v1/ai/chat-sessions",
            "/api/v1/ai/chat-sessions/{session_id}/messages",
            "/api/v1/ai/chat-sessions/{session_id}/messages"
        ]
    }

@app.get("/health")
async def health_check():
    return {"status": "healthy", "platform": "huggingface-spaces", "hf_available": HF_AVAILABLE}

@app.post("/api/v1/ai/chat-sessions", response_model=AIChatSessionResponse)
async def create_chat_session(session_data: AIChatSessionCreate):
    """Create a new AI chat session"""
    try:
        session = ai_service.create_chat_session(
            user_id=None,  # For Hugging Face Spaces, we don't have user authentication
            session_title=session_data.session_title
        )
        return session
    except Exception as e:
        logger.error(f"Error creating chat session: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to create chat session")

@app.get("/api/v1/ai/chat-sessions/{session_id}", response_model=AIChatSessionResponse)
async def get_chat_session(session_id: int):
    """Get an existing chat session"""
    try:
        session = ai_service.get_chat_session(session_id)
        if not session:
            raise HTTPException(status_code=404, detail="Chat session not found")
        return session
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting chat session: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to get chat session")

@app.post("/api/v1/ai/chat-sessions/{session_id}/messages", response_model=AIChatMessageResponse)
async def send_message(session_id: int, message_data: AIChatMessageCreate):
    """Send a message to the AI and get a response"""
    try:
        # Add user message to session
        user_message = ai_service.add_message_to_session(
            session_id=session_id,
            user_id=None,  # For Hugging Face Spaces, we don't have user authentication
            role=message_data.role,
            content=message_data.content
        )

        # Generate AI response based on the user message
        ai_response_text = ai_service.generate_ai_response(
            query=message_data.content,
            session_id=session_id,
            context_chapters=message_data.context_chapters
        )

        # Add AI response to session
        ai_message = ai_service.add_message_to_session(
            session_id=session_id,
            user_id=None,
            role="assistant",
            content=ai_response_text
        )

        return ai_message
    except Exception as e:
        logger.error(f"Error processing message: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to process message")

@app.get("/api/v1/ai/chat-sessions/{session_id}/messages", response_model=List[AIChatMessageResponse])
async def get_session_messages(session_id: int):
    """Get messages from a chat session"""
    try:
        messages = ai_service.get_session_history(session_id, limit=50)
        # Convert to the expected response format
        formatted_messages = []
        for msg in messages:
            formatted_messages.append(AIChatMessageResponse(
                id=msg["id"],
                session_id=msg["session_id"],
                user_id=msg.get("user_id"),
                message_type=msg["message_type"],
                content=msg["content"],
                role=msg["role"],
                sources=msg.get("sources", []),
                context_chapters=msg.get("context_chapters", []),
                timestamp=msg["timestamp"],
                is_grounding_validated=False
            ))
        return formatted_messages
    except Exception as e:
        logger.error(f"Error getting session messages: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to get session messages")

@app.post("/api/v1/ai/chat-sessions/{session_id}/generate-response")
async def generate_standalone_response(session_id: int, request: Request):
    """Generate an AI response to a query without adding it to the session"""
    try:
        # Parse the request body
        body = await request.json()
        query = body.get("query", "")
        context_chapters = body.get("context_chapters", [])

        response = ai_service.generate_ai_response(
            query=query,
            session_id=session_id,
            context_chapters=context_chapters
        )

        return {"response": response}
    except Exception as e:
        logger.error(f"Error generating response: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to generate response")

# Error handlers
@app.exception_handler(Exception)
async def global_exception_handler(request: Request, exc: Exception):
    """Global exception handler for uncaught exceptions"""
    error_id = f"hf_{datetime.utcnow().strftime('%Y%m%d_%H%M%S')}_{hash(str(exc)) % 10000:04d}"
    
    logger.error(
        f"Global exception {error_id}: {type(exc).__name__} - {str(exc)}",
        extra={
            "error_id": error_id,
            "url": str(request.url),
            "method": request.method,
            "timestamp": datetime.utcnow().isoformat()
        }
    )

    return JSONResponse(
        status_code=500,
        content={
            "error": True,
            "error_id": error_id,
            "message": "An unexpected error occurred. Please try again later.",
            "category": "system",
            "can_retry": True,
            "suggested_action": "Try again later. If the problem persists, contact support.",
            "timestamp": datetime.utcnow().isoformat()
        }
    )

@app.exception_handler(404)
async def not_found_handler(request: Request, exc: HTTPException):
    return JSONResponse(
        status_code=404,
        content={
            "error": True,
            "message": "The requested resource was not found.",
            "category": "not_found",
            "can_retry": False,
            "suggested_action": "Check the URL and try again."
        }
    )

# Gradio interface for Hugging Face Spaces
def create_gradio_interface():
    """Create a Gradio interface for the Hugging Face Space"""
    
    def predict(message, history, session_id):
        """Process a message and return AI response"""
        if not session_id:
            # Create a new session
            session_data = ai_service.create_chat_session(session_title="Gradio Session")
            session_id = session_data["id"]
        
        # Format message for the API
        message_data = AIChatMessageCreate(role="user", content=message)
        
        # Add user message to session
        user_message = ai_service.add_message_to_session(
            session_id=session_id,
            user_id=None,
            role="user",
            content=message
        )

        # Generate AI response
        ai_response_text = ai_service.generate_ai_response(
            query=message,
            session_id=session_id
        )

        # Add AI response to session
        ai_message = ai_service.add_message_to_session(
            session_id=session_id,
            user_id=None,
            role="assistant",
            content=ai_response_text
        )
        
        return ai_response_text, session_id
    
    with gr.Blocks() as demo:
        gr.Markdown("# Physical AI & Humanoid Robotics Textbook Assistant")
        gr.Markdown("Ask questions about Physical AI, Humanoid Robotics, and related topics!")
        
        session_id_state = gr.State(value=None)
        
        with gr.Row():
            with gr.Column(scale=1):
                new_session_btn = gr.Button("New Session")
            with gr.Column(scale=4):
                session_info = gr.Textbox(label="Session Info", interactive=False)
        
        chatbot = gr.Chatbot(height=400)
        msg = gr.Textbox(label="Your Question", placeholder="Ask about Physical AI, humanoid robots, ROS, Gazebo, etc...")
        submit_btn = gr.Button("Send")
        
        def start_new_session():
            session_data = ai_service.create_chat_session(session_title="Gradio Session")
            return session_data["id"], f"Started new session: {session_data['session_title']}"
        
        def reset_chat():
            return [], None, "New session will be created on first message"
        
        new_session_btn.click(
            start_new_session,
            outputs=[session_id_state, session_info]
        )
        
        submit_btn.click(
            predict,
            inputs=[msg, chatbot, session_id_state],
            outputs=[chatbot, session_id_state]
        )
        
        msg.submit(
            predict,
            inputs=[msg, chatbot, session_id_state],
            outputs=[chatbot, session_id_state]
        )
        
        gr.Button("Reset Chat").click(
            reset_chat,
            outputs=[chatbot, session_id_state, session_info]
        )
    
    return demo

# Create the Gradio interface
gradio_interface = create_gradio_interface()

if __name__ == "__main__":
    import uvicorn
    import argparse
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--use-gradio", action="store_true", help="Use Gradio interface instead of FastAPI")
    args = parser.parse_args()
    
    if args.use_gradio:
        # Run Gradio interface
        gradio_interface.launch(server_name="0.0.0.0", server_port=int(os.getenv("PORT", 7860)))
    else:
        # Run FastAPI
        uvicorn.run(app, host="0.0.0.0", port=int(os.getenv("PORT", 7860)))
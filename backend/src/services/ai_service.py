import os
import logging
from typing import List, Dict, Any, Optional
from datetime import datetime
from sqlalchemy.orm import Session

from ..models.ai_interaction import AIChatSession, AIChatMessage, AIInteractionLog
from ..models.chapter import Chapter
from ..lib.vector_store import vector_store, CHAPTER_CONTENT_COLLECTION
from ..lib.database import get_db

# Set up logging
logger = logging.getLogger(__name__)

# Import AI providers
try:
    from openai import OpenAI
    api_key = os.getenv("OPENAI_API_KEY")
    if api_key:
        openai_client = OpenAI(api_key=api_key)
        OPENAI_AVAILABLE = True
    else:
        logger.info("OpenAI API key not set.")
        openai_client = None
        OPENAI_AVAILABLE = False
except ImportError:
    logger.info("OpenAI library not available.")
    openai_client = None
    OPENAI_AVAILABLE = False

try:
    import google.generativeai as genai
    gemini_api_key = os.getenv("GEMINI_API_KEY")
    if gemini_api_key:
        genai.configure(api_key=gemini_api_key)
        gemini_model = genai.GenerativeModel('gemini-pro')
        GEMINI_AVAILABLE = True
    else:
        logger.info("GEMINI API key not set.")
        gemini_model = None
        GEMINI_AVAILABLE = False
except ImportError:
    logger.info("Google Generative AI library not available.")
    gemini_model = None
    GEMINI_AVAILABLE = False

# Determine primary AI provider based on availability
AI_PROVIDER = None
if OPENAI_AVAILABLE:
    AI_PROVIDER = "openai"
elif GEMINI_AVAILABLE:
    AI_PROVIDER = "gemini"
else:
    AI_PROVIDER = "mock"


class AIService:
    def __init__(self, db: Session):
        self.db = db

    def create_chat_session(self, user_id: Optional[int] = None, session_title: Optional[str] = None) -> AIChatSession:
        """
        Create a new AI chat session
        """
        session = AIChatSession(
            user_id=user_id,
            session_title=session_title,
            is_active=True
        )
        self.db.add(session)
        self.db.commit()
        self.db.refresh(session)
        return session

    def get_chat_session(self, session_id: int) -> Optional[AIChatSession]:
        """
        Get an existing chat session
        """
        return self.db.query(AIChatSession).filter(AIChatSession.id == session_id).first()

    def add_message_to_session(self, session_id: int, user_id: Optional[int], role: str, content: str) -> AIChatMessage:
        """
        Add a message to a chat session
        """
        message = AIChatMessage(
            session_id=session_id,
            user_id=user_id,
            message_type=role,
            role=role,
            content=content
        )
        self.db.add(message)
        self.db.commit()
        self.db.refresh(message)
        return message

    def search_content(self, query: str, limit: int = 5, filters: Optional[Dict[str, Any]] = None) -> List[Dict[str, Any]]:
        """
        Search for relevant content using vector store
        """
        try:
            # In a real implementation, we would use an embedding model to convert
            # the query to a vector. For now, we'll use a simple mock approach
            # that would be replaced with actual embedding generation.

            # This is where we would integrate with an embedding model like OpenAI's
            # text-embedding-ada-002, Google's embedding models, or a local model like SentenceTransformers
            query_vector = None

            if OPENAI_AVAILABLE:
                # Use OpenAI embeddings in a real implementation
                from openai import OpenAI
                client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

                response = client.embeddings.create(
                    input=query,
                    model="text-embedding-ada-002"
                )
                query_vector = response.data[0].embedding
            elif GEMINI_AVAILABLE:
                # For now, we'll use a mock approach for Gemini as it doesn't provide embedding API directly
                # In a real implementation, we might use Google's dedicated embedding models
                # or other approaches
                query_vector = [hash(c) % 1000 / 1000.0 for c in query[:1536]]  # Simple mock
            else:
                # Mock embedding for demonstration
                # In a real implementation without OpenAI, we might use:
                # from sentence_transformers import SentenceTransformer
                # model = SentenceTransformer('all-MiniLM-L6-v2')
                # query_vector = model.encode([query])[0].tolist()
                query_vector = [hash(c) % 1000 / 1000.0 for c in query[:1536]]  # Simple mock

            results = vector_store.search_vectors(
                collection_name=CHAPTER_CONTENT_COLLECTION,
                query_vector=query_vector,
                limit=limit,
                filters=filters
            )
            return results
        except Exception as e:
            logger.error(f"Error searching content: {str(e)}")
            return []

    def add_content_to_vector_store(self, chapter_id: int, content: str, section: str = ""):
        """
        Add chapter content to vector store for RAG retrieval
        """
        try:
            # In a real implementation, we would:
            # 1. Split content into chunks
            # 2. Generate embeddings for each chunk
            # 3. Store in vector database with metadata

            # For now, we'll just add the full content as a single vector
            # with chapter_id and section as metadata
            embedding = None

            if OPENAI_AVAILABLE:
                from openai import OpenAI
                client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

                # Generate embedding for the content
                response = client.embeddings.create(
                    input=content,
                    model="text-embedding-ada-002"
                )
                embedding = response.data[0].embedding
            elif GEMINI_AVAILABLE:
                # For now, we'll use a mock approach for Gemini as it doesn't provide embedding API directly
                # In a real implementation, we might use Google's dedicated embedding models
                embedding = [hash(c) % 1000 / 1000.0 for c in content[:1536]]  # Simple mock
            else:
                # Mock embedding for demonstration
                embedding = [hash(c) % 1000 / 1000.0 for c in content[:1536]]  # Simple mock

            # Prepare payload with metadata
            payload = {
                "content": content,
                "chapter_id": chapter_id,
                "section": section,
                "created_at": datetime.utcnow().isoformat()
            }

            # Upsert to vector store
            success = vector_store.upsert_vectors(
                collection_name=CHAPTER_CONTENT_COLLECTION,
                vectors=[embedding],
                payloads=[payload],
                ids=[chapter_id]  # Using chapter_id as vector ID for simplicity
            )

            return success
        except Exception as e:
            logger.error(f"Error adding content to vector store: {str(e)}")
            return False

    def get_relevant_content(self, query: str, context_chapters: Optional[List[int]] = None) -> List[Dict[str, Any]]:
        """
        Get relevant content for the AI to use as context
        """
        filters = {}
        if context_chapters:
            # In a real implementation, we would filter by chapter IDs
            # For now, we'll just search without specific chapter filtering
            pass

        return self.search_content(query, limit=5, filters=filters)

    def generate_ai_response(self, query: str, session_id: int, user_id: Optional[int] = None,
                           context_chapters: Optional[List[int]] = None) -> str:
        """
        Generate an AI response based on the query and context
        """
        try:
            # Get relevant content for context using vector search
            relevant_content = self.get_relevant_content(query, context_chapters)

            # Prepare the system message with context
            system_message = """You are an AI assistant for the Physical AI & Humanoid Robotics textbook.
            Your role is to help students understand concepts from the textbook.
            Base your responses on the textbook content provided in the context.
            Be accurate, helpful, and cite specific information when possible.
            If you don't have specific information from the textbook, acknowledge this limitation."""

            # Build context from retrieved content
            context_text = ""
            sources = []
            if relevant_content:
                context_text = "Relevant textbook content:\n"
                for item in relevant_content:
                    content_snippet = item.get("payload", {}).get("content", "")[:500]  # Limit snippet size
                    chapter_id = item.get("payload", {}).get("chapter_id", "unknown")
                    context_text += f"\nFrom Chapter {chapter_id}: {content_snippet}\n"
                    if chapter_id not in sources:
                        sources.append(chapter_id)
            else:
                context_text = "No specific textbook content found for this query. Use general knowledge about Physical AI and Humanoid Robotics."

            # Prepare the full user message with context
            user_context = f"{context_text}\n\nUser query: {query}"

            ai_response = ""
            model_used = "mock-model"

            if AI_PROVIDER == "openai" and OPENAI_AVAILABLE:
                # Use OpenAI API for real responses with proper context
                response = openai_client.chat.completions.create(
                    model=os.getenv("OPENAI_MODEL", "gpt-3.5-turbo"),
                    messages=[
                        {"role": "system", "content": system_message},
                        {"role": "user", "content": user_context}
                    ],
                    max_tokens=500,
                    temperature=0.7
                )
                ai_response = response.choices[0].message.content
                model_used = os.getenv("OPENAI_MODEL", "gpt-3.5-turbo")
            elif AI_PROVIDER == "gemini" and GEMINI_AVAILABLE:
                # Use Google Gemini API for responses
                full_prompt = f"{system_message}\n\n{user_context}"
                response = gemini_model.generate_content(
                    full_prompt,
                    generation_config={
                        "max_output_tokens": 500,
                        "temperature": 0.7
                    }
                )
                ai_response = response.text if response.text else "I couldn't generate a response for your query."
                model_used = "gemini-pro"
            else:
                # Enhanced mock response that considers retrieved content
                ai_response = self._generate_mock_response_with_context(query, context_chapters, context_text)
                model_used = "mock-model"

            # Log the interaction with sources
            self._log_interaction(user_id, session_id, query, ai_response, sources, model_used)

            return ai_response
        except Exception as e:
            logger.error(f"Error generating AI response: {str(e)}")
            return "Sorry, I encountered an error processing your request. Please try again."

    def _generate_mock_response(self, query: str, context_chapters: Optional[List[int]] = None) -> str:
        """
        Generate a mock response when OpenAI is not available
        """
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

        else:
            return f"""Based on the textbook content, Physical AI and Humanoid Robotics involve complex interactions between cognitive systems and physical environments.
            The field addresses challenges like real-time processing, uncertainty management, and energy efficiency.
            In the context of your query about "{query[:50]}{'...' if len(query) > 50 else ''}", these principles form the foundation for understanding embodied AI systems."""

    def _generate_mock_response_with_context(self, query: str, context_chapters: Optional[List[int]] = None, context_text: str = "") -> str:
        """
        Generate a mock response that considers retrieved context
        """
        query_lower = query.lower()

        # Check if the context contains relevant information
        context_lower = context_text.lower()

        if 'physical ai' in query_lower or 'embodiment' in query_lower:
            if 'principles' in context_lower or 'core' in context_lower:
                return f"""Based on the textbook content: Physical AI represents a paradigm shift where cognitive systems are embodied in physical form and interact with the real world.
                Unlike traditional AI that operates in virtual environments, Physical AI systems must navigate the complexities of physics, dynamics, and real-world uncertainty.
                The core principles include:
                1. Embodiment: Intelligence emerges from the interaction between an agent and its physical environment
                2. Real-time Processing: Systems must respond to environmental changes within strict temporal constraints
                3. Uncertainty Management: Dealing with sensor noise, actuator limitations, and environmental variability
                4. Energy Efficiency: Optimizing for real-world power constraints"""
            else:
                return """Physical AI represents a paradigm shift where cognitive systems are embodied in physical form and interact with the real world.
                The core principles include embodiment, real-time processing, uncertainty management, and energy efficiency."""

        elif 'humanoid' in query_lower or 'robot' in query_lower:
            if 'application' in context_lower or 'robotics' in context_lower:
                return f"""Based on the textbook content: Humanoid robotics is a key application area for Physical AI. Humanoid robots must perform complex tasks in human environments,
                requiring sophisticated locomotion, manipulation, and interaction capabilities. Key applications include:
                - Locomotion: Walking, running, and navigating complex terrains
                - Manipulation: Grasping objects with human-like dexterity
                - Interaction: Communicating and collaborating with humans
                - Adaptation: Learning from physical interactions to improve performance"""
            else:
                return """Humanoid robotics is a key application area for Physical AI, involving locomotion, manipulation, and interaction capabilities."""

        elif 'challenge' in query_lower or 'difficulty' in query_lower:
            if 'challenge' in context_lower or 'difficulty' in context_lower:
                return f"""Based on the textbook content: The main technical challenges in Physical AI include:
                1. Sim-to-Real Gap: Bridging the difference between simulation and real-world performance
                2. Safety: Ensuring safe operation around humans and environments
                3. Scalability: Developing systems that can operate reliably across diverse scenarios
                4. Learning Efficiency: Acquiring new skills with minimal physical interaction"""
            else:
                return """The main challenges in Physical AI include the sim-to-real gap, safety, scalability, and learning efficiency."""

        else:
            return f"""Based on the textbook content: Physical AI and Humanoid Robotics involve complex interactions between cognitive systems and physical environments.
            The field addresses challenges like real-time processing, uncertainty management, and energy efficiency.
            Your query about "{query[:50]}{'...' if len(query) > 50 else ''}" relates to these fundamental concepts."""

    def _log_interaction(self, user_id: Optional[int], session_id: int, input_text: str, output_text: str, sources: Optional[List[int]] = None, model_used: str = "mock-model"):
        """
        Log the AI interaction for analysis and improvement
        """
        try:
            log_entry = AIInteractionLog(
                user_id=user_id,
                session_id=session_id,
                interaction_type='chat',
                input_text=input_text,
                output_text=output_text,
                ai_model_used=model_used,
                sources=sources,  # Add sources to the log
                created_at=datetime.utcnow()
            )
            self.db.add(log_entry)
            self.db.commit()
        except Exception as e:
            logger.error(f"Error logging interaction: {str(e)}")

    def get_session_history(self, session_id: int, limit: int = 50) -> List[AIChatMessage]:
        """
        Get the message history for a session
        """
        return self.db.query(AIChatMessage)\
                     .filter(AIChatMessage.session_id == session_id)\
                     .order_by(AIChatMessage.timestamp.desc())\
                     .limit(limit)\
                     .all()


# Convenience function to get AI service
def get_ai_service(db: Session) -> AIService:
    return AIService(db)
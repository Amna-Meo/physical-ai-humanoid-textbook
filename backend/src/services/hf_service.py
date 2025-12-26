"""
Hugging Face integration for Physical AI & Humanoid Robotics Textbook
This module integrates Hugging Face models into the backend API
"""
import os
import logging
from typing import List, Dict, Any, Optional
from datetime import datetime
from transformers import pipeline, AutoTokenizer, AutoModelForCausalLM
import torch
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Set up logging
logger = logging.getLogger(__name__)

class HuggingFaceAIService:
    """
    Service class to handle Hugging Face model interactions
    """
    def __init__(self):
        self.model_name = os.getenv("HF_MODEL_NAME", "mistralai/Mistral-7B-Instruct-v0.2")
        self.tokenizer = None
        self.model = None
        self.generator = None
        self._initialize_model()
        
    def _initialize_model(self):
        """
        Initialize the Hugging Face model and tokenizer
        """
        try:
            logger.info(f"Initializing model: {self.model_name}")
            
            # Check if we're using a local model or need to download
            if os.path.exists(self.model_name):
                # Load from local path
                self.tokenizer = AutoTokenizer.from_pretrained(self.model_name)
                self.model = AutoModelForCausalLM.from_pretrained(
                    self.model_name,
                    torch_dtype=torch.float16 if torch.cuda.is_available() else torch.float32,
                    device_map="auto" if torch.cuda.is_available() else None
                )
            else:
                # Load from Hugging Face Hub
                self.tokenizer = AutoTokenizer.from_pretrained(self.model_name)
                self.model = AutoModelForCausalLM.from_pretrained(
                    self.model_name,
                    torch_dtype=torch.float16 if torch.cuda.is_available() else torch.float32,
                    device_map="auto" if torch.cuda.is_available() else None,
                    trust_remote_code=True
                )
            
            # Create text generation pipeline
            self.generator = pipeline(
                "text-generation",
                model=self.model,
                tokenizer=self.tokenizer,
                torch_dtype=torch.float16 if torch.cuda.is_available() else torch.float32,
                device_map="auto" if torch.cuda.is_available() else None
            )
            
            logger.info(f"Successfully initialized model: {self.model_name}")
        except Exception as e:
            logger.error(f"Error initializing model {self.model_name}: {str(e)}")
            # Fallback to a simpler approach if model loading fails
            self.generator = None

    def generate_text(self, prompt: str, max_length: int = 500, temperature: float = 0.7) -> str:
        """
        Generate text using the Hugging Face model
        """
        if self.generator is None:
            # Fallback response if model is not available
            return self._fallback_response(prompt)
        
        try:
            # Format the prompt for the model if it's Mistral
            formatted_prompt = f"<s>[INST] {prompt} [/INST]"
            
            # Generate response
            result = self.generator(
                formatted_prompt,
                max_length=max_length,
                temperature=temperature,
                do_sample=True,
                pad_token_id=self.tokenizer.eos_token_id,
                truncation=True
            )
            
            # Extract the generated text
            generated_text = result[0]['generated_text']
            
            # Remove the original prompt from the result to get just the AI response
            response = generated_text[len(formatted_prompt):].strip()
            
            return response
        except Exception as e:
            logger.error(f"Error generating text: {str(e)}")
            return self._fallback_response(prompt)

    def _fallback_response(self, query: str) -> str:
        """
        Fallback response when Hugging Face model is not available
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

    def generate_with_context(self, query: str, context: Optional[str] = None) -> str:
        """
        Generate a response with additional context
        """
        if context:
            full_prompt = f"Context: {context}\n\nQuestion: {query}\n\nPlease provide a detailed response based on the context."
        else:
            full_prompt = query
            
        return self.generate_text(full_prompt, max_length=700, temperature=0.6)

# Global instance of the Hugging Face service
hf_service = HuggingFaceAIService()

def get_hf_service():
    """
    Function to get the Hugging Face service instance
    """
    return hf_service
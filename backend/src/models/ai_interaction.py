from sqlalchemy import Column, Integer, String, Text, DateTime, Boolean, ForeignKey, JSON
from sqlalchemy.sql import func
from sqlalchemy.orm import relationship
from backend.src.lib.database import Base
from pydantic import BaseModel
from typing import Optional, List, Dict, Any
from datetime import datetime


class AIChatSession(Base):
    __tablename__ = "ai_chat_sessions"

    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(Integer, ForeignKey("users.id"), nullable=True)  # Nullable for anonymous sessions
    session_title = Column(String, nullable=True)  # Auto-generated or user-provided
    is_active = Column(Boolean, default=True)
    session_metadata = Column(JSON, nullable=True)  # For session context, preferences, etc.
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())


class AIChatMessage(Base):
    __tablename__ = "ai_chat_messages"

    id = Column(Integer, primary_key=True, index=True)
    session_id = Column(Integer, ForeignKey("ai_chat_sessions.id"), nullable=False)
    user_id = Column(Integer, ForeignKey("users.id"), nullable=True)  # Who sent the message
    message_type = Column(String, nullable=False)  # 'user', 'assistant', 'system'
    content = Column(Text, nullable=False)
    role = Column(String, nullable=False)  # 'user', 'assistant', 'system' (for OpenAI API)
    sources = Column(JSON, nullable=True)  # Sources used for the response (chapter IDs, citations, etc.)
    timestamp = Column(DateTime(timezone=True), server_default=func.now())

    # For RAG context
    context_chapters = Column(JSON, nullable=True)  # Chapters referenced in the conversation
    is_grounding_validated = Column(Boolean, default=False)  # Whether response is grounded in textbook


class AIInteractionLog(Base):
    __tablename__ = "ai_interaction_logs"

    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(Integer, ForeignKey("users.id"), nullable=True)
    session_id = Column(Integer, ForeignKey("ai_chat_sessions.id"), nullable=False)
    interaction_type = Column(String, nullable=False)  # 'chat', 'question', 'explanation', 'summary', etc.
    input_text = Column(Text, nullable=False)
    output_text = Column(Text, nullable=False)
    ai_model_used = Column(String, nullable=False)  # Name of the AI model used
    tokens_input = Column(Integer, nullable=True)  # Number of input tokens
    tokens_output = Column(Integer, nullable=True)  # Number of output tokens
    response_time_ms = Column(Integer, nullable=True)  # Response time in milliseconds
    is_accurate = Column(Boolean, nullable=True)  # Whether response was accurate (for evaluation)
    feedback_score = Column(Integer, nullable=True)  # User feedback (1-5)
    feedback_text = Column(Text, nullable=True)  # User feedback text
    sources = Column(JSON, nullable=True)  # Sources used for the response (chapter IDs, citations, etc.)
    created_at = Column(DateTime(timezone=True), server_default=func.now())


class AIPreference(Base):
    __tablename__ = "ai_preferences"

    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(Integer, ForeignKey("users.id"), nullable=False)
    preference_key = Column(String, nullable=False)  # 'difficulty_level', 'explanation_style', etc.
    preference_value = Column(String, nullable=False)  # 'beginner', 'detailed', etc.
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

    __table_args__ = (
        # Ensure unique user-preference pairs
        # sqlalchemy.UniqueConstraint('user_id', 'preference_key'),
    )


# Pydantic models for API
class AIChatSessionBase(BaseModel):
    user_id: Optional[int] = None
    session_title: Optional[str] = None
    session_metadata: Optional[dict] = None


class AIChatSessionCreate(AIChatSessionBase):
    pass


class AIChatSessionResponse(AIChatSessionBase):
    id: int
    is_active: bool
    created_at: datetime
    updated_at: Optional[datetime] = None

    class Config:
        from_attributes = True


class AIChatMessageBase(BaseModel):
    session_id: int
    message_type: str
    content: str
    role: str
    sources: Optional[List[Dict[str, Any]]] = None
    context_chapters: Optional[List[int]] = None


class AIChatMessageCreate(AIChatMessageBase):
    pass


class AIChatMessageResponse(AIChatMessageBase):
    id: int
    user_id: Optional[int] = None
    timestamp: datetime
    is_grounding_validated: bool

    class Config:
        from_attributes = True


class AIInteractionLogBase(BaseModel):
    user_id: Optional[int] = None
    session_id: int
    interaction_type: str
    input_text: str
    output_text: str
    ai_model_used: str
    tokens_input: Optional[int] = None
    tokens_output: Optional[int] = None
    response_time_ms: Optional[int] = None
    is_accurate: Optional[bool] = None
    feedback_score: Optional[int] = None
    feedback_text: Optional[str] = None
    sources: Optional[List[int]] = None


class AIInteractionLogResponse(AIInteractionLogBase):
    id: int
    created_at: datetime

    class Config:
        from_attributes = True


class AIPreferenceBase(BaseModel):
    user_id: int
    preference_key: str
    preference_value: str


class AIPreferenceResponse(AIPreferenceBase):
    id: int
    created_at: datetime
    updated_at: Optional[datetime] = None

    class Config:
        from_attributes = True
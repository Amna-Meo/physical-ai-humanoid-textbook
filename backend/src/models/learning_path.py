from sqlalchemy import Column, Integer, String, Text, DateTime, Boolean, ForeignKey, JSON
from sqlalchemy.sql import func
from sqlalchemy.orm import relationship
from backend.src.lib.database import Base
from pydantic import BaseModel
from typing import Optional, List
from datetime import datetime


class LearningPath(Base):
    __tablename__ = "learning_paths"

    id = Column(Integer, primary_key=True, index=True)
    title = Column(String, nullable=False)
    slug = Column(String, unique=True, nullable=False, index=True)
    description = Column(Text, nullable=True)
    is_active = Column(Boolean, default=True)
    is_personalized = Column(Boolean, default=False)  # Whether path is personalized
    difficulty_level = Column(String, nullable=True)  # beginner, intermediate, advanced
    estimated_duration_hours = Column(Integer, nullable=True)

    # Metadata
    path_metadata = Column(JSON, nullable=True)  # For additional path configuration
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())


class LearningPathStep(Base):
    __tablename__ = "learning_path_steps"

    id = Column(Integer, primary_key=True, index=True)
    learning_path_id = Column(Integer, ForeignKey("learning_paths.id"), nullable=False)
    step_number = Column(Integer, nullable=False)
    title = Column(String, nullable=False)
    description = Column(Text, nullable=True)
    content_type = Column(String, nullable=False)  # 'chapter', 'exercise', 'quiz', 'simulation', etc.
    content_id = Column(Integer, nullable=False)  # ID of the referenced content
    required = Column(Boolean, default=True)
    estimated_duration_minutes = Column(Integer, nullable=True)

    created_at = Column(DateTime(timezone=True), server_default=func.now())


class UserLearningPath(Base):
    __tablename__ = "user_learning_paths"

    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(Integer, ForeignKey("users.id"), nullable=False)
    learning_path_id = Column(Integer, ForeignKey("learning_paths.id"), nullable=False)
    status = Column(String, default="not_started")  # not_started, in_progress, completed
    current_step = Column(Integer, default=0)  # Current step index
    progress_percentage = Column(Integer, default=0)
    started_at = Column(DateTime(timezone=True), server_default=func.now())
    completed_at = Column(DateTime(timezone=True), nullable=True)

    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())


class UserLearningPathProgress(Base):
    __tablename__ = "user_learning_path_progress"

    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(Integer, ForeignKey("users.id"), nullable=False)
    learning_path_id = Column(Integer, ForeignKey("learning_paths.id"), nullable=False)
    step_id = Column(Integer, ForeignKey("learning_path_steps.id"), nullable=False)
    status = Column(String, default="not_started")  # not_started, in_progress, completed
    score = Column(Integer, nullable=True)  # For scored steps like quizzes
    completed_at = Column(DateTime(timezone=True), nullable=True)

    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())


# Pydantic models for API
class LearningPathBase(BaseModel):
    title: str
    slug: str
    description: Optional[str] = None
    is_active: bool = True
    is_personalized: bool = False
    difficulty_level: Optional[str] = None
    estimated_duration_hours: Optional[int] = None
    path_metadata: Optional[dict] = None


class LearningPathCreate(LearningPathBase):
    pass


class LearningPathUpdate(BaseModel):
    title: Optional[str] = None
    description: Optional[str] = None
    is_active: Optional[bool] = None
    is_personalized: Optional[bool] = None
    difficulty_level: Optional[str] = None
    estimated_duration_hours: Optional[int] = None
    path_metadata: Optional[dict] = None


class LearningPathResponse(LearningPathBase):
    id: int
    created_at: datetime
    updated_at: Optional[datetime] = None

    class Config:
        from_attributes = True


class LearningPathStepBase(BaseModel):
    learning_path_id: int
    step_number: int
    title: str
    description: Optional[str] = None
    content_type: str
    content_id: int
    required: bool = True
    estimated_duration_minutes: Optional[int] = None


class LearningPathStepResponse(LearningPathStepBase):
    id: int
    created_at: datetime

    class Config:
        from_attributes = True


class UserLearningPathBase(BaseModel):
    user_id: int
    learning_path_id: int


class UserLearningPathUpdate(BaseModel):
    status: Optional[str] = None
    current_step: Optional[int] = None
    progress_percentage: Optional[int] = None


class UserLearningPathResponse(UserLearningPathBase):
    id: int
    status: str
    current_step: int
    progress_percentage: int
    started_at: datetime
    completed_at: Optional[datetime] = None
    created_at: datetime
    updated_at: Optional[datetime] = None

    class Config:
        from_attributes = True
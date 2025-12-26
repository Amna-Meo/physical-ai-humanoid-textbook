from sqlalchemy import Column, Integer, String, DateTime, Boolean, Text
from sqlalchemy.sql import func
from ..lib.database import Base
from pydantic import BaseModel
from typing import Optional
from datetime import datetime


class User(Base):
    __tablename__ = "users"

    id = Column(Integer, primary_key=True, index=True)
    email = Column(String, unique=True, index=True, nullable=False)
    username = Column(String, unique=True, index=True, nullable=False)
    full_name = Column(String, nullable=True)
    hashed_password = Column(String, nullable=False)
    is_active = Column(Boolean, default=True)
    is_verified = Column(Boolean, default=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

    # Professional background information
    professional_background = Column(Text, nullable=True)  # For personalization
    learning_preferences = Column(Text, nullable=True)     # JSON string for preferences
    timezone = Column(String, nullable=True)
    language_preference = Column(String, default="en")


# Pydantic models for API
class UserBase(BaseModel):
    email: str
    username: str
    full_name: Optional[str] = None


class UserCreate(UserBase):
    password: str
    professional_background: Optional[str] = None
    learning_preferences: Optional[str] = None
    timezone: Optional[str] = None
    language_preference: Optional[str] = "en"


class UserUpdate(BaseModel):
    full_name: Optional[str] = None
    professional_background: Optional[str] = None
    learning_preferences: Optional[str] = None
    timezone: Optional[str] = None
    language_preference: Optional[str] = None


class UserResponse(UserBase):
    id: int
    is_active: bool
    is_verified: bool
    created_at: datetime
    updated_at: Optional[datetime] = None

    class Config:
        from_attributes = True
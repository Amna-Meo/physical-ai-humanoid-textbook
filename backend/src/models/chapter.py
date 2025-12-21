from sqlalchemy import Column, Integer, String, Text, DateTime, Boolean, ForeignKey, JSON
from sqlalchemy.sql import func
from sqlalchemy.orm import relationship
from backend.src.lib.database import Base
from pydantic import BaseModel
from typing import Optional, List
from datetime import datetime


class Chapter(Base):
    __tablename__ = "chapters"

    id = Column(Integer, primary_key=True, index=True)
    title = Column(String, nullable=False)
    slug = Column(String, unique=True, nullable=False, index=True)
    content = Column(Text, nullable=False)  # Markdown or HTML content
    description = Column(Text, nullable=True)
    chapter_number = Column(Integer, nullable=False)
    word_count = Column(Integer, nullable=True)
    reading_time_minutes = Column(Integer, nullable=True)  # Estimated reading time

    # Metadata for AI processing
    chapter_metadata = Column(JSON, nullable=True)  # For concept tags, learning objectives, etc.
    citations = Column(JSON, nullable=True)  # List of citations in JSON format

    # Status and publishing
    is_published = Column(Boolean, default=False)
    is_ai_optimized = Column(Boolean, default=False)  # Whether content is optimized for AI
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

    # Foreign key for book relationship (if needed)
    # book_id = Column(Integer, ForeignKey("books.id"), nullable=True)


class ChapterContentBlock(Base):
    __tablename__ = "chapter_content_blocks"

    id = Column(Integer, primary_key=True, index=True)
    chapter_id = Column(Integer, ForeignKey("chapters.id"), nullable=False)
    block_type = Column(String, nullable=False)  # 'text', 'code', 'image', 'equation', 'exercise', etc.
    content = Column(Text, nullable=False)
    order_index = Column(Integer, nullable=False)
    block_metadata = Column(JSON, nullable=True)  # Additional metadata for specific block types
    created_at = Column(DateTime(timezone=True), server_default=func.now())


# Pydantic models for API
class ChapterBase(BaseModel):
    title: str
    slug: str
    content: str
    description: Optional[str] = None
    chapter_number: int
    is_published: bool = False


class ChapterCreate(ChapterBase):
    pass


class ChapterUpdate(BaseModel):
    title: Optional[str] = None
    content: Optional[str] = None
    description: Optional[str] = None
    chapter_number: Optional[int] = None
    is_published: Optional[bool] = None
    chapter_metadata: Optional[dict] = None
    citations: Optional[list] = None


class ChapterResponse(ChapterBase):
    id: int
    word_count: Optional[int] = None
    reading_time_minutes: Optional[int] = None
    is_ai_optimized: bool
    created_at: datetime
    updated_at: Optional[datetime] = None

    class Config:
        from_attributes = True


class ChapterContentBlockBase(BaseModel):
    chapter_id: int
    block_type: str
    content: str
    order_index: int
    block_metadata: Optional[dict] = None


class ChapterContentBlockResponse(ChapterContentBlockBase):
    id: int
    created_at: datetime

    class Config:
        from_attributes = True
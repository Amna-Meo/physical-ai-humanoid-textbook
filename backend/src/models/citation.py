from sqlalchemy import Column, Integer, String, Text, DateTime, Boolean, ForeignKey, JSON
from sqlalchemy.sql import func
from sqlalchemy.orm import relationship
from ..lib.database import Base
from pydantic import BaseModel
from typing import Optional, List, Dict, Any
from datetime import datetime


class Citation(Base):
    __tablename__ = "citations"

    id = Column(Integer, primary_key=True, index=True)
    citation_key = Column(String, unique=True, nullable=False)  # e.g., "author2023title"
    title = Column(String, nullable=False)
    authors = Column(Text, nullable=True)  # Comma-separated authors
    journal = Column(String, nullable=True)
    volume = Column(String, nullable=True)
    issue = Column(String, nullable=True)
    pages = Column(String, nullable=True)
    year = Column(Integer, nullable=True)
    publisher = Column(String, nullable=True)
    doi = Column(String, nullable=True)  # Digital Object Identifier
    url = Column(String, nullable=True)
    citation_type = Column(String, nullable=False)  # 'book', 'journal', 'conference', 'thesis', 'web', etc.
    raw_citation = Column(Text, nullable=False)  # Full citation in raw format
    apa_formatted = Column(Text, nullable=False)  # APA 7th edition formatted citation
    bibtex_formatted = Column(Text, nullable=True)  # BibTeX format
    is_verified = Column(Boolean, default=False)  # Whether citation has been verified
    citation_metadata = Column(JSON, nullable=True)  # Additional metadata like abstract, keywords, etc.
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())


class ChapterCitation(Base):
    __tablename__ = "chapter_citations"

    id = Column(Integer, primary_key=True, index=True)
    chapter_id = Column(Integer, ForeignKey("chapters.id"), nullable=False)
    citation_id = Column(Integer, ForeignKey("citations.id"), nullable=False)
    citation_context = Column(Text, nullable=True)  # Context where citation is used
    page_reference = Column(String, nullable=True)  # Specific page reference if applicable
    created_at = Column(DateTime(timezone=True), server_default=func.now())

    __table_args__ = (
        # Ensure unique chapter-citation pairs
        # sqlalchemy.UniqueConstraint('chapter_id', 'citation_id'),
    )


class CitationVerification(Base):
    __tablename__ = "citation_verifications"

    id = Column(Integer, primary_key=True, index=True)
    citation_id = Column(Integer, ForeignKey("citations.id"), nullable=False)
    verified_by = Column(Integer, ForeignKey("users.id"), nullable=True)  # Who verified
    verification_method = Column(String, nullable=False)  # 'manual', 'cross_reference', 'doi_lookup', etc.
    is_accurate = Column(Boolean, nullable=False)  # Whether citation is accurate
    notes = Column(Text, nullable=True)
    verified_at = Column(DateTime(timezone=True), server_default=func.now())


# Pydantic models for API
class CitationBase(BaseModel):
    citation_key: str
    title: str
    citation_type: str
    raw_citation: str
    apa_formatted: str
    authors: Optional[str] = None
    journal: Optional[str] = None
    volume: Optional[str] = None
    issue: Optional[str] = None
    pages: Optional[str] = None
    year: Optional[int] = None
    publisher: Optional[str] = None
    doi: Optional[str] = None
    url: Optional[str] = None
    bibtex_formatted: Optional[str] = None
    citation_metadata: Optional[dict] = None


class CitationCreate(CitationBase):
    pass


class CitationUpdate(BaseModel):
    title: Optional[str] = None
    authors: Optional[str] = None
    journal: Optional[str] = None
    volume: Optional[str] = None
    issue: Optional[str] = None
    pages: Optional[str] = None
    year: Optional[int] = None
    publisher: Optional[str] = None
    doi: Optional[str] = None
    url: Optional[str] = None
    raw_citation: Optional[str] = None
    apa_formatted: Optional[str] = None
    bibtex_formatted: Optional[str] = None
    is_verified: Optional[bool] = None
    citation_metadata: Optional[dict] = None


class CitationResponse(CitationBase):
    id: int
    is_verified: bool
    created_at: datetime
    updated_at: Optional[datetime] = None

    class Config:
        from_attributes = True


class ChapterCitationBase(BaseModel):
    chapter_id: int
    citation_id: int
    citation_context: Optional[str] = None
    page_reference: Optional[str] = None


class ChapterCitationResponse(ChapterCitationBase):
    id: int
    created_at: datetime

    class Config:
        from_attributes = True


class CitationVerificationBase(BaseModel):
    citation_id: int
    verification_method: str
    is_accurate: bool
    notes: Optional[str] = None


class CitationVerificationResponse(CitationVerificationBase):
    id: int
    verified_by: Optional[int] = None
    verified_at: datetime

    class Config:
        from_attributes = True
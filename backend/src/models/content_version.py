from sqlalchemy import Column, Integer, String, Text, DateTime, ForeignKey, Boolean, JSON
from sqlalchemy.sql import func
from sqlalchemy.orm import relationship
from ..lib.database import Base


class ContentVersion(Base):
    """
    Model for tracking content versions and changes
    """
    __tablename__ = "content_versions"

    id = Column(Integer, primary_key=True, index=True)
    content_id = Column(Integer, nullable=False)  # Can be chapter_id, etc.
    content_type = Column(String, nullable=False)  # 'chapter', 'user_note', etc.
    version_number = Column(Integer, nullable=False)
    title = Column(String, nullable=True)
    content = Column(Text, nullable=True)
    summary = Column(Text, nullable=True)  # Summary of changes
    author_id = Column(Integer, ForeignKey("users.id"), nullable=True)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    is_active = Column(Boolean, default=True)
    change_type = Column(String, default="update")  # create, update, delete
    change_metadata = Column(JSON, default=dict)  # Additional metadata about the change

    # Relationships
    author = relationship("User", back_populates="content_versions")


class ContentChangeLog(Base):
    """
    Model for tracking all content changes with detailed information
    """
    __tablename__ = "content_change_logs"

    id = Column(Integer, primary_key=True, index=True)
    content_id = Column(Integer, nullable=False)
    content_type = Column(String, nullable=False)
    version_id = Column(Integer, ForeignKey("content_versions.id"), nullable=True)
    user_id = Column(Integer, ForeignKey("users.id"), nullable=True)
    action = Column(String, nullable=False)  # created, updated, deleted, published, etc.
    field_changed = Column(String, nullable=True)  # Which field was changed
    old_value = Column(Text, nullable=True)  # Previous value
    new_value = Column(Text, nullable=True)  # New value
    change_summary = Column(Text, nullable=True)  # Human-readable summary of change
    timestamp = Column(DateTime(timezone=True), server_default=func.now())
    ip_address = Column(String, nullable=True)  # IP of the user making the change
    user_agent = Column(Text, nullable=True)  # Browser/agent info

    # Relationships
    user = relationship("User", back_populates="content_change_logs")
    version = relationship("ContentVersion", back_populates="change_logs")


# Update User model to include relationships
# This would normally be added to the existing user model file
def extend_user_model():
    """
    This function shows how to extend the User model with relationships to content versions
    In practice, this would be added to the user.py model file
    """
    pass
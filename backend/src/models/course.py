from sqlalchemy import Column, Integer, String, Text, DateTime, Boolean, ForeignKey, JSON
from sqlalchemy.sql import func
from sqlalchemy.orm import relationship
from backend.src.lib.database import Base
from pydantic import BaseModel
from typing import Optional, List
from datetime import datetime


class Course(Base):
    __tablename__ = "courses"

    id = Column(Integer, primary_key=True, index=True)
    title = Column(String, nullable=False)
    slug = Column(String, unique=True, nullable=False, index=True)
    description = Column(Text, nullable=True)
    instructor_id = Column(Integer, ForeignKey("users.id"), nullable=False)  # Creator/instructor
    is_active = Column(Boolean, default=True)
    is_published = Column(Boolean, default=False)

    # Course settings
    start_date = Column(DateTime(timezone=True), nullable=True)
    end_date = Column(DateTime(timezone=True), nullable=True)
    enrollment_open = Column(Boolean, default=True)

    # Metadata
    course_metadata = Column(JSON, nullable=True)  # For additional course configuration
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())


class CourseEnrollment(Base):
    __tablename__ = "course_enrollments"

    id = Column(Integer, primary_key=True, index=True)
    course_id = Column(Integer, ForeignKey("courses.id"), nullable=False)
    user_id = Column(Integer, ForeignKey("users.id"), nullable=False)
    enrollment_date = Column(DateTime(timezone=True), server_default=func.now())
    enrollment_status = Column(String, default="active")  # active, dropped, completed
    role = Column(String, default="student")  # student, teaching_assistant

    created_at = Column(DateTime(timezone=True), server_default=func.now())


class CourseChapter(Base):
    __tablename__ = "course_chapters"

    id = Column(Integer, primary_key=True, index=True)
    course_id = Column(Integer, ForeignKey("courses.id"), nullable=False)
    chapter_id = Column(Integer, ForeignKey("chapters.id"), nullable=False)
    order_index = Column(Integer, nullable=False)  # Order of chapters in the course
    is_required = Column(Boolean, default=True)
    due_date = Column(DateTime(timezone=True), nullable=True)  # Optional assignment due date

    created_at = Column(DateTime(timezone=True), server_default=func.now())


class CourseAssignment(Base):
    __tablename__ = "course_assignments"

    id = Column(Integer, primary_key=True, index=True)
    course_id = Column(Integer, ForeignKey("courses.id"), nullable=False)
    title = Column(String, nullable=False)
    description = Column(Text, nullable=True)
    chapter_id = Column(Integer, ForeignKey("chapters.id"), nullable=True)  # Associated chapter
    assignment_type = Column(String, default="reading")  # reading, quiz, project, etc.
    points = Column(Integer, default=100)
    due_date = Column(DateTime(timezone=True), nullable=True)
    is_published = Column(Boolean, default=False)

    assignment_metadata = Column(JSON, nullable=True)  # Assignment-specific settings
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())


class UserCourseProgress(Base):
    __tablename__ = "user_course_progress"

    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(Integer, ForeignKey("users.id"), nullable=False)
    course_id = Column(Integer, ForeignKey("courses.id"), nullable=False)
    chapter_id = Column(Integer, ForeignKey("chapters.id"), nullable=False)
    progress_percentage = Column(Integer, default=0)  # 0-100
    completed_at = Column(DateTime(timezone=True), nullable=True)
    score = Column(Integer, nullable=True)  # For graded components

    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())


class CourseAnalytics(Base):
    __tablename__ = "course_analytics"

    id = Column(Integer, primary_key=True, index=True)
    course_id = Column(Integer, ForeignKey("courses.id"), nullable=False)
    metric_name = Column(String, nullable=False)  # "completion_rate", "engagement", etc.
    metric_value = Column(Integer, nullable=True)
    metric_data = Column(JSON, nullable=True)  # For complex metrics
    recorded_at = Column(DateTime(timezone=True), server_default=func.now())


# Pydantic models for API
class CourseBase(BaseModel):
    title: str
    slug: str
    description: Optional[str] = None
    instructor_id: int
    is_active: bool = True
    is_published: bool = False
    start_date: Optional[datetime] = None
    end_date: Optional[datetime] = None
    enrollment_open: bool = True
    course_metadata: Optional[dict] = None


class CourseCreate(CourseBase):
    pass


class CourseUpdate(BaseModel):
    title: Optional[str] = None
    description: Optional[str] = None
    is_active: Optional[bool] = None
    is_published: Optional[bool] = None
    start_date: Optional[datetime] = None
    end_date: Optional[datetime] = None
    enrollment_open: Optional[bool] = None
    course_metadata: Optional[dict] = None


class CourseResponse(CourseBase):
    id: int
    created_at: datetime
    updated_at: Optional[datetime] = None

    class Config:
        from_attributes = True


class CourseEnrollmentBase(BaseModel):
    course_id: int
    user_id: int
    enrollment_status: str = "active"
    role: str = "student"


class CourseEnrollmentResponse(CourseEnrollmentBase):
    id: int
    enrollment_date: datetime
    created_at: datetime

    class Config:
        from_attributes = True


class CourseChapterBase(BaseModel):
    course_id: int
    chapter_id: int
    order_index: int
    is_required: bool = True
    due_date: Optional[datetime] = None


class CourseChapterResponse(CourseChapterBase):
    id: int
    created_at: datetime

    class Config:
        from_attributes = True


class CourseAssignmentBase(BaseModel):
    course_id: int
    title: str
    description: Optional[str] = None
    chapter_id: Optional[int] = None
    assignment_type: str = "reading"
    points: int = 100
    due_date: Optional[datetime] = None
    is_published: bool = False
    assignment_metadata: Optional[dict] = None


class CourseAssignmentUpdate(BaseModel):
    title: Optional[str] = None
    description: Optional[str] = None
    chapter_id: Optional[int] = None
    assignment_type: Optional[str] = None
    points: Optional[int] = None
    due_date: Optional[datetime] = None
    is_published: Optional[bool] = None
    assignment_metadata: Optional[dict] = None


class CourseAssignmentResponse(CourseAssignmentBase):
    id: int
    created_at: datetime
    updated_at: Optional[datetime] = None

    class Config:
        from_attributes = True


class UserCourseProgressBase(BaseModel):
    user_id: int
    course_id: int
    chapter_id: int
    progress_percentage: int = 0
    score: Optional[int] = None


class UserCourseProgressResponse(UserCourseProgressBase):
    id: int
    completed_at: Optional[datetime] = None
    created_at: datetime
    updated_at: Optional[datetime] = None

    class Config:
        from_attributes = True
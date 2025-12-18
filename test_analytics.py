#!/usr/bin/env python3
"""
Test script for course analytics functionality
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

# Use in-memory SQLite for testing instead of PostgreSQL
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from backend.src.models.course import Base

# Create an in-memory SQLite database for testing
DATABASE_URL = "sqlite:///:memory:"
engine = create_engine(DATABASE_URL, echo=False)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
from backend.src.models.course import Course, CourseEnrollment, UserCourseProgress, CourseChapter
from backend.src.models.user import User, UserCreate
from backend.src.models.chapter import Chapter
from backend.src.services.course_service import get_course_service
from backend.src.services.user_service import get_user_service

def test_analytics():
    """Test the analytics functionality"""
    # Create all tables
    Base.metadata.create_all(bind=engine)

    db = SessionLocal()

    try:
        print("Testing course analytics functionality...")

        # Create test data using user service
        user_service = get_user_service(db)

        # Create a test user (instructor)
        instructor_data = UserCreate(
            username="test_instructor",
            email="instructor@test.com",
            full_name="Test Instructor",
            password="pass123"  # Using a short password to avoid bcrypt issues
        )
        instructor = user_service.create_user(instructor_data)

        # Create a test user (student)
        student_data = UserCreate(
            username="test_student",
            email="student@test.com",
            full_name="Test Student",
            password="pass123"  # Using a short password to avoid bcrypt issues
        )
        student = user_service.create_user(student_data)

        # Create a test chapter
        chapter = Chapter(
            title="Test Chapter",
            slug="test-chapter",
            content="This is a test chapter content",
            chapter_number=1,
            word_count=1000,
            reading_time_minutes=5,
            is_published=True,
            is_ai_optimized=True
        )
        db.add(chapter)
        db.commit()
        db.refresh(chapter)

        # Create a test course
        course = Course(
            title="Test Course",
            slug="test-course",
            description="This is a test course",
            instructor_id=instructor.id,
            is_active=True,
            is_published=True
        )
        db.add(course)
        db.commit()
        db.refresh(course)

        # Add chapter to course
        course_chapter = CourseChapter(
            course_id=course.id,
            chapter_id=chapter.id,
            order_index=0,
            is_required=True
        )
        db.add(course_chapter)
        db.commit()

        # Enroll student in course
        enrollment = CourseEnrollment(
            course_id=course.id,
            user_id=student.id,
            role="student"
        )
        db.add(enrollment)
        db.commit()

        # Add some progress for the student
        progress = UserCourseProgress(
            user_id=student.id,
            course_id=course.id,
            chapter_id=chapter.id,
            progress_percentage=75,
            score=85
        )
        db.add(progress)
        db.commit()

        print(f"Created test data: Instructor ID {instructor.id}, Course ID {course.id}, Student ID {student.id}")

        # Test analytics
        course_service = get_course_service(db)

        # Test course completion rate
        completion_rate = course_service.get_course_completion_rate(course.id)
        print(f"Course completion rate: {completion_rate}%")

        # Test course analytics
        course_analytics = course_service.get_course_analytics(course.id)
        if course_analytics:
            print("Course analytics:")
            print(f"  - Total students: {course_analytics['total_students']}")
            print(f"  - Completion rate: {course_analytics['completion_rate']}%")
            print(f"  - Average progress: {course_analytics['avg_progress']}%")
            print(f"  - Engagement rate: {course_analytics['engagement_rate']}%")
            print(f"  - Total assignments: {course_analytics['total_assignments']}")
            print(f"  - Assignment completion rate: {course_analytics['assignment_completion_rate']}%")
            print(f"  - Average assignment score: {course_analytics['avg_assignment_score']}")
        else:
            print("Failed to get course analytics")

        # Test instructor analytics
        instructor_analytics = course_service.get_instructor_analytics(instructor.id)
        if instructor_analytics:
            print("Instructor analytics:")
            print(f"  - Total courses: {instructor_analytics['total_courses']}")
            print(f"  - Total students across courses: {instructor_analytics['total_students_across_courses']}")
            print(f"  - Average course completion rate: {instructor_analytics['avg_course_completion_rate']}%")
        else:
            print("Failed to get instructor analytics")

        print("Analytics tests completed successfully!")

    except Exception as e:
        print(f"Error during testing: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Clean up test data
        try:
            db.query(UserCourseProgress).filter(
                UserCourseProgress.user_id.in_([student.id if 'student' in locals() else 0])
            ).delete()
            db.query(CourseChapter).filter(
                CourseChapter.course_id.in_([course.id if 'course' in locals() else 0])
            ).delete()
            db.query(CourseEnrollment).filter(
                CourseEnrollment.course_id.in_([course.id if 'course' in locals() else 0])
            ).delete()
            db.query(Course).filter(
                Course.instructor_id.in_([instructor.id if 'instructor' in locals() else 0])
            ).delete()
            db.query(Chapter).delete()
            db.query(User).filter(
                User.username.in_(['test_instructor', 'test_student'])
            ).delete()
            db.commit()
        except:
            pass  # Ignore cleanup errors
        db.close()

if __name__ == "__main__":
    test_analytics()
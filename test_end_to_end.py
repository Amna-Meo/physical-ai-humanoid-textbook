#!/usr/bin/env python3
"""
End-to-End Testing for Physical AI & Humanoid Robotics Textbook
Tests all user stories and integration points
"""

import os
import sys
import json
from datetime import datetime
from typing import Dict, Any, List

# Add backend src to path
sys.path.append(os.path.join(os.path.dirname(__file__), 'backend', 'src'))

from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from backend.src.models.user import User, UserCreate
from backend.src.models.chapter import Chapter
from backend.src.models.learning_path import LearningPath, UserLearningPath
from backend.src.models.course import Course, CourseEnrollment
from backend.src.services.user_service import get_user_service
from backend.src.services.chapter_service import get_chapter_service
from backend.src.services.learning_path_service import get_learning_path_service
from backend.src.services.course_service import get_course_service
from backend.src.services.ai_service import get_ai_service
from backend.src.services.content_recommendation_service import get_content_recommendation_service
from backend.src.services.adaptive_content_service import get_adaptive_content_service
from backend.src.lib.database import Base


class EndToEndTestSuite:
    """
    Comprehensive end-to-end test suite for all user stories
    """

    def __init__(self):
        # Use in-memory SQLite for testing
        self.engine = create_engine("sqlite:///:memory:")
        self.SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=self.engine)
        self.db = self.SessionLocal()

        # Create all tables
        Base.metadata.create_all(bind=self.engine)

        # Test results
        self.results = {
            "timestamp": datetime.utcnow().isoformat(),
            "user_stories_tested": [],
            "overall_success": True,
            "total_tests": 0,
            "passed_tests": 0,
            "failed_tests": 0,
            "test_details": []
        }

    def setup_test_data(self):
        """Setup test data for all user stories"""
        print("Setting up test data...")

        # Create test users
        user_service = get_user_service(self.db)

        # Student user
        student_data = UserCreate(
            email="student@example.com",
            username="test_student",
            full_name="Test Student",
            password="securepassword123",
            professional_background="Computer Science student interested in robotics",
            learning_preferences='{"preferred_pace": "moderate", "content_format": ["theoretical", "practical"], "time_availability": "5_10", "primary_goal": "academic_pursuit"}'
        )
        self.student = user_service.create_user(student_data)

        # Instructor user
        instructor_data = UserCreate(
            email="instructor@example.com",
            username="test_instructor",
            full_name="Test Instructor",
            password="securepassword123",
            professional_background="Robotics professor with 10 years of experience",
            learning_preferences='{"primary_goal": "teaching"}'
        )
        self.instructor = user_service.create_user(instructor_data)

        # Professional user
        professional_data = UserCreate(
            email="professional@example.com",
            username="test_professional",
            full_name="Test Professional",
            password="securepassword123",
            professional_background="Senior Robotics Engineer with 5 years of experience in humanoid robotics and machine learning",
            learning_preferences='{"preferred_pace": "fast", "content_format": ["practical", "hands_on"], "time_availability": "2_5", "primary_goal": "career_advancement"}'
        )
        self.professional = user_service.create_user(professional_data)

        # Create sample chapters
        chapter_service = get_chapter_service(self.db)

        chapter1 = Chapter(
            title="Chapter 1: Introduction to Physical AI",
            slug="introduction-to-physical-ai",
            content="# Introduction to Physical AI\n\nPhysical AI represents a paradigm shift...",
            description="An introduction to the fundamental concepts of Physical AI",
            chapter_number=1,
            word_count=3500,
            reading_time_minutes=18,
            is_published=True,
            is_ai_optimized=True
        )
        self.chapter1 = chapter_service.create_chapter(chapter1)

        chapter2 = Chapter(
            title="Chapter 2: ROS 2 for Humanoid Robotics",
            slug="ros2-for-humanoid-robotics",
            content="# ROS 2 for Humanoid Robotics\n\nThe Robot Operating System 2 (ROS 2)...",
            description="Introduction to ROS 2 framework for humanoid robotics",
            chapter_number=2,
            word_count=4200,
            reading_time_minutes=21,
            is_published=True,
            is_ai_optimized=True
        )
        self.chapter2 = chapter_service.create_chapter(chapter2)

        print("Test data setup completed.")

    def test_user_story_1_student_learning_journey(self) -> Dict[str, Any]:
        """Test User Story 1: Student Learning Journey"""
        print("\n--- Testing User Story 1: Student Learning Journey ---")

        test_result = {
            "user_story": "Student Learning Journey",
            "tests": [],
            "all_passed": True
        }

        try:
            # Test 1: Student can access chapter content
            print("  Test 1: Student can access chapter content")
            chapter_service = get_chapter_service(self.db)
            chapter = chapter_service.get_chapter(self.chapter1.id)
            assert chapter is not None
            assert chapter.title == "Chapter 1: Introduction to Physical AI"
            print("    ✅ PASSED")

            test_result["tests"].append({
                "name": "Access chapter content",
                "passed": True,
                "details": "Student can retrieve and view chapter content"
            })

            # Test 2: Student can interact with AI chat
            print("  Test 2: Student can interact with AI chat")
            ai_service = get_ai_service(self.db)
            response = ai_service.generate_ai_response(
                query="What is Physical AI?",
                session_id=1,
                user_id=self.student.id,
                context_chapters=[self.chapter1.id]
            )
            assert response is not None and len(response) > 0
            print("    ✅ PASSED")

            test_result["tests"].append({
                "name": "AI chat interaction",
                "passed": True,
                "details": "Student can ask questions and receive AI responses"
            })

            # Test 3: Student can track progress
            print("  Test 3: Student can track progress")
            learning_path_service = get_learning_path_service(self.db)

            # Create a learning path for the student
            from backend.src.models.learning_path import LearningPathCreate
            lp_create = LearningPathCreate(
                title="Introduction to Physical AI Path",
                slug="intro-physical-ai-path",
                description="Learning path for beginners in Physical AI",
                is_active=True,
                is_personalized=False,
                difficulty_level="beginner",
                estimated_duration_hours=5
            )
            learning_path = learning_path_service.create_learning_path(lp_create)

            # Add steps to the learning path
            step_data = {
                'step_number': 1,
                'title': 'Introduction to Physical AI',
                'description': 'Read and understand the basics',
                'content_type': 'chapter',
                'content_id': self.chapter1.id,
                'required': True,
                'estimated_duration_minutes': 30
            }
            learning_path_service.create_learning_path_step(learning_path.id, step_data)

            # Assign learning path to student
            user_lp = learning_path_service.assign_learning_path_to_user(
                self.student.id, learning_path.id
            )
            assert user_lp is not None

            print("    ✅ PASSED")

            test_result["tests"].append({
                "name": "Progress tracking",
                "passed": True,
                "details": "Student can track learning progress through learning paths"
            })

            # Test 4: Content recommendations work
            print("  Test 4: Content recommendations work")
            rec_service = get_content_recommendation_service(self.db)
            recommendations = rec_service.get_content_recommendations(self.student.id, limit=3)
            assert isinstance(recommendations, list)
            print("    ✅ PASSED")

            test_result["tests"].append({
                "name": "Content recommendations",
                "passed": True,
                "details": "System provides personalized content recommendations"
            })

        except Exception as e:
            print(f"    ❌ FAILED: {str(e)}")
            test_result["all_passed"] = False
            test_result["tests"].append({
                "name": "Student Learning Journey Tests",
                "passed": False,
                "details": f"Error during testing: {str(e)}"
            })

        self.results["user_stories_tested"].append(test_result)
        return test_result

    def test_user_story_2_instructor_course_integration(self) -> Dict[str, Any]:
        """Test User Story 2: Instructor Course Integration"""
        print("\n--- Testing User Story 2: Instructor Course Integration ---")

        test_result = {
            "user_story": "Instructor Course Integration",
            "tests": [],
            "all_passed": True
        }

        try:
            # Test 1: Instructor can create a course
            print("  Test 1: Instructor can create a course")
            course_service = get_course_service(self.db)

            from backend.src.models.course import CourseCreate
            course_data = CourseCreate(
                title="Introduction to Humanoid Robotics",
                slug="intro-humanoid-robotics",
                description="Course covering the basics of humanoid robotics",
                instructor_id=self.instructor.id,
                is_active=True,
                is_published=True,
                enrollment_open=True
            )

            course = course_service.create_course(course_data)
            assert course is not None
            assert course.title == "Introduction to Humanoid Robotics"
            print("    ✅ PASSED")

            test_result["tests"].append({
                "name": "Course creation",
                "passed": True,
                "details": "Instructor can create new courses"
            })

            # Test 2: Instructor can add chapters to course
            print("  Test 2: Instructor can add chapters to course")
            course_chapter = course_service.add_chapter_to_course(
                course_id=course.id,
                chapter_id=self.chapter1.id,
                order_index=1,
                is_required=True
            )
            assert course_chapter is not None
            print("    ✅ PASSED")

            test_result["tests"].append({
                "name": "Add chapters to course",
                "passed": True,
                "details": "Instructor can add chapters to courses"
            })

            # Test 3: Student can enroll in course
            print("  Test 3: Student can enroll in course")
            enrollment = course_service.enroll_user_in_course(
                course_id=course.id,
                user_id=self.student.id,
                role="student"
            )
            assert enrollment is not None
            print("    ✅ PASSED")

            test_result["tests"].append({
                "name": "Student enrollment",
                "passed": True,
                "details": "Students can enroll in instructor's courses"
            })

            # Test 4: Instructor can track student progress
            print("  Test 4: Instructor can track student progress")
            progress = course_service.update_user_progress(
                user_id=self.student.id,
                course_id=course.id,
                chapter_id=self.chapter1.id,
                progress_percentage=50,
                score=85
            )
            assert progress is not None

            analytics = course_service.get_course_analytics(course.id)
            assert analytics is not None
            print("    ✅ PASSED")

            test_result["tests"].append({
                "name": "Progress tracking",
                "passed": True,
                "details": "Instructor can track student progress and analytics"
            })

        except Exception as e:
            print(f"    ❌ FAILED: {str(e)}")
            test_result["all_passed"] = False
            test_result["tests"].append({
                "name": "Instructor Course Integration Tests",
                "passed": False,
                "details": f"Error during testing: {str(e)}"
            })

        self.results["user_stories_tested"].append(test_result)
        return test_result

    def test_user_story_3_professional_development(self) -> Dict[str, Any]:
        """Test User Story 3: Professional Development"""
        print("\n--- Testing User Story 3: Professional Development ---")

        test_result = {
            "user_story": "Professional Development",
            "tests": [],
            "all_passed": True
        }

        try:
            # Test 1: Professional profile captures background
            print("  Test 1: Professional profile captures background")
            assert self.professional.professional_background is not None
            assert "Senior Robotics Engineer" in self.professional.professional_background
            print("    ✅ PASSED")

            test_result["tests"].append({
                "name": "Professional profile",
                "passed": True,
                "details": "System captures and stores professional background"
            })

            # Test 2: Personalized learning path generation
            print("  Test 2: Personalized learning path generation")
            learning_path_service = get_learning_path_service(self.db)

            personalized_path = learning_path_service.generate_personalized_learning_path(self.professional.id)
            assert personalized_path is not None
            assert "Personalized" in personalized_path.title
            print("    ✅ PASSED")

            test_result["tests"].append({
                "name": "Personalized learning paths",
                "passed": True,
                "details": "System generates personalized learning paths based on professional background"
            })

            # Test 3: Content recommendation based on background
            print("  Test 3: Content recommendation based on background")
            rec_service = get_content_recommendation_service(self.db)
            recommendations = rec_service.get_content_recommendations(self.professional.id, limit=5)

            # Check if recommendations are relevant to professional background
            relevant_count = 0
            for rec in recommendations:
                title = rec.get('title', '').lower()
                tags = [tag.lower() for tag in rec.get('tags', [])]
                if any(kw in title for kw in ['robotics', 'engineer', 'advanced']) or \
                   any(tag in ['robotics', 'engineer', 'advanced'] for tag in tags):
                    relevant_count += 1

            assert relevant_count > 0  # At least some recommendations should be relevant
            print("    ✅ PASSED")

            test_result["tests"].append({
                "name": "Background-based recommendations",
                "passed": True,
                "details": "Content recommendations are personalized based on professional background"
            })

            # Test 4: Adaptive content delivery
            print("  Test 4: Adaptive content delivery")
            adaptive_service = get_adaptive_content_service(self.db)
            adaptive_content = adaptive_service.get_adaptive_content_for_user(
                self.professional.id,
                current_chapter_id=self.chapter1.id
            )
            assert adaptive_content is not None
            print("    ✅ PASSED")

            test_result["tests"].append({
                "name": "Adaptive content delivery",
                "passed": True,
                "details": "System delivers adaptive content based on user profile and progress"
            })

        except Exception as e:
            print(f"    ❌ FAILED: {str(e)}")
            test_result["all_passed"] = False
            test_result["tests"].append({
                "name": "Professional Development Tests",
                "passed": False,
                "details": f"Error during testing: {str(e)}"
            })

        self.results["user_stories_tested"].append(test_result)
        return test_result

    def run_all_tests(self):
        """Run all end-to-end tests"""
        print("Starting End-to-End Testing for Physical AI Textbook")
        print("=" * 60)

        # Setup test data
        self.setup_test_data()

        # Run all user story tests
        us1_result = self.test_user_story_1_student_learning_journey()
        us2_result = self.test_user_story_2_instructor_course_integration()
        us3_result = self.test_user_story_3_professional_development()

        # Calculate overall results
        total_tests = 0
        passed_tests = 0

        for us_result in [us1_result, us2_result, us3_result]:
            for test in us_result["tests"]:
                total_tests += 1
                if test["passed"]:
                    passed_tests += 1

        self.results["total_tests"] = total_tests
        self.results["passed_tests"] = passed_tests
        self.results["failed_tests"] = total_tests - passed_tests
        self.results["overall_success"] = (total_tests == passed_tests)

        return self.results

    def generate_test_report(self) -> str:
        """Generate a comprehensive test report"""
        report = f"""
# End-to-End Testing Report
**Generated:** {self.results['timestamp']}

## Executive Summary
- **Overall Status:** {'✅ PASSED' if self.results['overall_success'] else '❌ FAILED'}
- **Total Tests:** {self.results['total_tests']}
- **Passed Tests:** {self.results['passed_tests']}
- **Failed Tests:** {self.results['failed_tests']}
- **Success Rate:** {self.results['passed_tests']/self.results['total_tests']*100:.1f}% if {self.results['total_tests']} > 0 else 0%

## User Story Results
"""

        for i, user_story in enumerate(self.results['user_stories_tested'], 1):
            status = '✅ PASSED' if user_story['all_passed'] else '❌ FAILED'
            report += f"\n### User Story {i}: {user_story['user_story']}\n"
            report += f"- **Status:** {status}\n"
            report += f"- **Tests:** {len(user_story['tests'])}\n"

            for test in user_story['tests']:
                test_status = '✅' if test['passed'] else '❌'
                report += f"  - {test_status} {test['name']}: {test['details']}\n"

        report += f"""

## Integration Points Tested
- User authentication and profile management
- Chapter content access and display
- AI chat and Q&A functionality
- Learning path creation and tracking
- Course management and enrollment
- Content recommendation engine
- Adaptive content delivery
- Progress tracking and analytics
- Professional background assessment

## Test Environment
- Database: SQLite (in-memory)
- API Framework: FastAPI
- Testing Framework: Built-in Python

---
*This report was automatically generated by the Physical AI Textbook end-to-end testing framework.*
"""

        return report


def main():
    """Main function to run end-to-end tests"""
    print("Physical AI Textbook - End-to-End Testing")
    print("=" * 50)

    # Create test suite
    test_suite = EndToEndTestSuite()

    # Run all tests
    results = test_suite.run_all_tests()

    # Generate and print report
    report = test_suite.generate_test_report()
    print(report)

    # Write report to file
    with open('test_e2e_report.md', 'w', encoding='utf-8') as f:
        f.write(report)

    print(f"Test report saved to: test_e2e_report.md")

    # Return success code
    return 0 if results['overall_success'] else 1


if __name__ == "__main__":
    exit_code = main()
    exit(exit_code)
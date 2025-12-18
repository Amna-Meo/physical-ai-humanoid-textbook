#!/usr/bin/env python3
"""
Test script to validate the professional development personalization features (T056)
This script tests the complete workflow for professional development features.
"""

import os
import sys
import json
from datetime import datetime
from sqlalchemy import create_engine, Column, Integer, String, Text, DateTime, Boolean, ForeignKey
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker, Session
from sqlalchemy.pool import QueuePool

# Add the backend src directory to the path so we can import our modules
sys.path.append(os.path.join(os.path.dirname(__file__), 'backend', 'src'))

# Import models and services
from backend.src.models.user import User, UserCreate
from backend.src.models.chapter import Chapter
from backend.src.models.learning_path import LearningPath, LearningPathStep
from backend.src.services.user_service import get_user_service
from backend.src.services.learning_path_service import get_learning_path_service
from backend.src.services.content_recommendation_service import get_content_recommendation_service
from backend.src.services.adaptive_content_service import get_adaptive_content_service
from backend.src.lib.database import get_db

# Create an in-memory SQLite database for testing
DATABASE_URL = "sqlite:///:memory:"
engine = create_engine(
    DATABASE_URL,
    poolclass=QueuePool,
    pool_size=5,
    max_overflow=10,
    pool_pre_ping=True,
    pool_recycle=300,
    echo=False
)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)


def create_test_user(db: Session):
    """Create a test user with professional background"""
    user_service = get_user_service(db)

    # Create user with professional background
    user_data = UserCreate(
        email="professional@example.com",
        username="test_professional",
        full_name="Test Professional",
        password="pass123",
        professional_background="Senior Robotics Engineer with 5 years of experience in humanoid robotics and machine learning. Specialized in control systems and AI planning.",
        learning_preferences=json.dumps({
            "preferred_pace": "moderate",
            "content_format": ["theoretical", "practical"],
            "learning_style": "balanced",
            "time_availability": "5_10",
            "primary_goal": "career_advancement"
        }),
        timezone="UTC",
        language_preference="en"
    )

    user = user_service.create_user(user_data)
    print(f"Created test user with ID: {user.id}")
    return user


def create_sample_chapters(db: Session):
    """Create sample chapters for testing"""
    chapters = [
        Chapter(
            title="Introduction to Physical AI",
            slug="introduction-to-physical-ai",
            content="This chapter introduces the fundamental concepts of Physical AI...",
            description="An introductory chapter on Physical AI concepts",
            chapter_number=1,
            word_count=2500,
            reading_time_minutes=25,
            is_published=True,
            is_ai_optimized=True
        ),
        Chapter(
            title="Robot Control Systems",
            slug="robot-control-systems",
            content="This chapter covers control systems for robots, including PID controllers...",
            description="Control systems for robotic applications",
            chapter_number=2,
            word_count=3200,
            reading_time_minutes=32,
            is_published=True,
            is_ai_optimized=True
        ),
        Chapter(
            title="AI Planning for Robotics",
            slug="ai-planning-for-robotics",
            content="This chapter explores AI planning algorithms for robotic systems...",
            description="Planning algorithms for intelligent robotic systems",
            chapter_number=3,
            word_count=2800,
            reading_time_minutes=28,
            is_published=True,
            is_ai_optimized=True
        ),
        Chapter(
            title="Advanced Humanoid Locomotion",
            slug="advanced-humanoid-locomotion",
            content="This chapter covers advanced techniques for humanoid robot walking...",
            description="Advanced locomotion for humanoid robots",
            chapter_number=4,
            word_count=3500,
            reading_time_minutes=35,
            is_published=True,
            is_ai_optimized=True
        )
    ]

    for chapter in chapters:
        db.add(chapter)

    db.commit()

    # Refresh to get IDs
    for chapter in chapters:
        db.refresh(chapter)

    print(f"Created {len(chapters)} sample chapters")
    return chapters


def create_sample_learning_paths(db: Session):
    """Create sample learning paths for testing"""
    learning_paths = [
        LearningPath(
            title="Robotics Fundamentals",
            slug="robotics-fundamentals",
            description="Learn the fundamentals of robotics including kinematics, dynamics, and control",
            is_active=True,
            is_personalized=False,
            difficulty_level="beginner",
            estimated_duration_hours=20
        ),
        LearningPath(
            title="AI for Robotics",
            slug="ai-for-robotics",
            description="Explore AI techniques specifically for robotics applications",
            is_active=True,
            is_personalized=False,
            difficulty_level="intermediate",
            estimated_duration_hours=25
        ),
        LearningPath(
            title="Humanoid Robotics",
            slug="humanoid-robotics",
            description="Specialized path for humanoid robot development and control",
            is_active=True,
            is_personalized=False,
            difficulty_level="advanced",
            estimated_duration_hours=30
        )
    ]

    for path in learning_paths:
        db.add(path)

    db.commit()

    # Refresh to get IDs
    for path in learning_paths:
        db.refresh(path)

    print(f"Created {len(learning_paths)} sample learning paths")
    return learning_paths


def test_content_recommendations(db: Session, user_id: int):
    """Test content recommendation engine"""
    print("\n--- Testing Content Recommendations ---")

    recommendation_service = get_content_recommendation_service(db)

    try:
        recommendations = recommendation_service.get_content_recommendations(user_id, limit=5)
        print(f"✅ Got {len(recommendations)} content recommendations")

        for i, rec in enumerate(recommendations[:3], 1):
            print(f"   {i}. {rec['title']} (Score: {rec['relevance_score']:.2f})")

        return len(recommendations) > 0
    except Exception as e:
        print(f"❌ Error getting content recommendations: {str(e)}")
        return False


def test_learning_path_recommendations(db: Session, user_id: int):
    """Test learning path recommendation engine"""
    print("\n--- Testing Learning Path Recommendations ---")

    recommendation_service = get_content_recommendation_service(db)

    try:
        recommendations = recommendation_service.get_learning_path_recommendations(user_id, limit=3)
        print(f"✅ Got {len(recommendations)} learning path recommendations")

        for i, rec in enumerate(recommendations, 1):
            print(f"   {i}. {rec['title']} (Score: {rec['relevance_score']:.2f})")

        return len(recommendations) > 0
    except Exception as e:
        print(f"❌ Error getting learning path recommendations: {str(e)}")
        return False


def test_personalized_learning_path_generation(db: Session, user_id: int):
    """Test personalized learning path generation"""
    print("\n--- Testing Personalized Learning Path Generation ---")

    learning_path_service = get_learning_path_service(db)

    try:
        personalized_path = learning_path_service.generate_personalized_learning_path(user_id)
        print(f"✅ Generated personalized learning path: {personalized_path.title}")
        print(f"   Difficulty: {personalized_path.difficulty_level}")
        print(f"   Estimated duration: {personalized_path.estimated_duration_hours} hours")

        # Check if steps were added based on background
        steps = learning_path_service.get_learning_path_steps(personalized_path.id)
        print(f"   Steps in path: {len(steps)}")

        for step in steps:
            print(f"     - {step.title}")

        return True
    except Exception as e:
        print(f"❌ Error generating personalized learning path: {str(e)}")
        return False


def test_adaptive_content_delivery(db: Session, user_id: int):
    """Test adaptive content delivery based on user progress"""
    print("\n--- Testing Adaptive Content Delivery ---")

    adaptive_service = get_adaptive_content_service(db)

    try:
        adaptive_content = adaptive_service.get_adaptive_content_for_user(user_id)
        print("✅ Got adaptive content recommendations")

        print(f"   Recommended content: {len(adaptive_content['recommended_content'])}")
        print(f"   Performance insights available: {bool(adaptive_content['performance_insights'])}")
        print(f"   Learning path adjustments: {len(adaptive_content['learning_path_adjustments'])}")
        print(f"   Support resources: {len(adaptive_content['support_resources'])}")

        # Check if recommendations are personalized
        if adaptive_content['recommended_content']:
            first_rec = adaptive_content['recommended_content'][0]
            print(f"   First recommendation: {first_rec['title']} (Score: {first_rec['adaptation_score']:.2f})")

        return True
    except Exception as e:
        print(f"❌ Error getting adaptive content: {str(e)}")
        return False


def test_background_assessment_integration(db: Session, user_id: int):
    """Test that background assessment properly influences recommendations"""
    print("\n--- Testing Background Assessment Integration ---")

    user_service = get_user_service(db)
    user = user_service.get_user_by_id(user_id)

    if not user.professional_background:
        print("❌ User professional background not found")
        return False

    print(f"✅ User background: {user.professional_background[:100]}...")
    print(f"✅ Learning preferences: {user.learning_preferences}")

    # Test that content is relevant to background
    recommendation_service = get_content_recommendation_service(db)
    recommendations = recommendation_service.get_content_recommendations(user_id, limit=3)

    # Check if recommendations are relevant to robotics/AI
    relevant_count = 0
    for rec in recommendations:
        title = rec['title'].lower()
        desc = rec['description'].lower()
        if any(keyword in title + desc for keyword in ['robot', 'ai', 'control', 'planning']):
            relevant_count += 1

    print(f"✅ {relevant_count}/{len(recommendations)} recommendations are relevant to robotics/AI")

    return relevant_count > 0


def test_professional_development_workflow():
    """Test the complete professional development workflow"""
    print("Testing Professional Development Personalization Features (T056)")
    print("=" * 60)

    # Create database session
    db = SessionLocal()

    try:
        # Create all tables
        from backend.src.models.user import User
        from backend.src.models.chapter import Chapter
        from backend.src.models.learning_path import LearningPath, LearningPathStep, UserLearningPath, UserLearningPathProgress
        from backend.src.models.ai_interaction import AIChatSession, AIChatMessage
        from backend.src.lib.database import Base
        Base.metadata.create_all(bind=engine)

        # Create test user with professional background
        user = create_test_user(db)

        # Create sample content
        chapters = create_sample_chapters(db)
        learning_paths = create_sample_learning_paths(db)

        # Test all professional development features
        test_results = []

        test_results.append(("Content Recommendations", test_content_recommendations(db, user.id)))
        test_results.append(("Learning Path Recommendations", test_learning_path_recommendations(db, user.id)))
        test_results.append(("Personalized Learning Path Generation", test_personalized_learning_path_generation(db, user.id)))
        test_results.append(("Adaptive Content Delivery", test_adaptive_content_delivery(db, user.id)))
        test_results.append(("Background Assessment Integration", test_background_assessment_integration(db, user.id)))

        # Overall validation
        print("\n" + "=" * 60)
        print("PROFESSIONAL DEVELOPMENT WORKFLOW VALIDATION RESULTS:")

        all_passed = True
        for test_name, passed in test_results:
            status = "✅ PASSED" if passed else "❌ FAILED"
            print(f"{test_name}: {status}")
            if not passed:
                all_passed = False

        print(f"\n{'✅ PROFESSIONAL DEVELOPMENT PERSONALIZATION FEATURES (T056): PASSED' if all_passed else '❌ PROFESSIONAL DEVELOPMENT PERSONALIZATION FEATURES (T056): FAILED'}")

        if all_passed:
            print("Professional development features are working correctly with personalization based on user background.")
        else:
            print("Some professional development features are not working correctly.")

        return all_passed

    except Exception as e:
        print(f"\n❌ Error during testing: {str(e)}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        db.close()


def main():
    """Main function to run the professional development tests"""
    success = test_professional_development_workflow()

    if success:
        print("\n🎉 ALL PROFESSIONAL DEVELOPMENT PERSONALIZATION TESTS PASSED!")
        print("Phase 5: User Story 3 - Professional Development is complete.")
    else:
        print("\n❌ SOME PROFESSIONAL DEVELOPMENT TESTS FAILED!")
        print("Please review the errors above.")

    return success


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
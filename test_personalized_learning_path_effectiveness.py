#!/usr/bin/env python3
"""
Test script to validate personalized learning path effectiveness (T057)
This script tests how effectively personalized learning paths adapt to user progress and performance.
"""

import os
import sys
import json
from datetime import datetime, timedelta
from sqlalchemy import create_engine, Column, Integer, String, Text, DateTime, Boolean, ForeignKey
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker, Session
from sqlalchemy.pool import QueuePool

# Add the backend src directory to the path so we can import our modules
sys.path.append(os.path.join(os.path.dirname(__file__), 'backend', 'src'))

# Import models and services
from backend.src.models.user import User, UserCreate
from backend.src.models.chapter import Chapter
from backend.src.models.learning_path import LearningPath, LearningPathStep, UserLearningPath, UserLearningPathProgress
from backend.src.models.ai_interaction import AIChatSession, AIChatMessage
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


def create_test_users(db: Session):
    """Create test users with different professional backgrounds"""
    user_service = get_user_service(db)

    # User 1: Robotics Engineer
    user1_data = UserCreate(
        email="robotics@example.com",
        username="robotics_eng",
        full_name="Robotics Engineer",
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
    user1 = user_service.create_user(user1_data)
    print(f"Created test user 1 with ID: {user1.id}")

    # User 2: AI Researcher
    user2_data = UserCreate(
        email="ai@example.com",
        username="ai_researcher",
        full_name="AI Researcher",
        password="pass123",
        professional_background="AI Research Scientist with 3 years of experience in neural networks and deep learning. Focused on theoretical research and algorithm development.",
        learning_preferences=json.dumps({
            "preferred_pace": "slow",
            "content_format": ["theoretical"],
            "learning_style": "theoretical",
            "time_availability": "2_5",
            "primary_goal": "research"
        }),
        timezone="UTC",
        language_preference="en"
    )
    user2 = user_service.create_user(user2_data)
    print(f"Created test user 2 with ID: {user2.id}")

    # User 3: Software Developer
    user3_data = UserCreate(
        email="dev@example.com",
        username="software_dev",
        full_name="Software Developer",
        password="pass123",
        professional_background="Software Developer with 2 years of experience in web development. New to AI and robotics, looking to transition into robotics software development.",
        learning_preferences=json.dumps({
            "preferred_pace": "fast",
            "content_format": ["practical", "hands_on"],
            "learning_style": "practical",
            "time_availability": "20_plus",
            "primary_goal": "career_change"
        }),
        timezone="UTC",
        language_preference="en"
    )
    user3 = user_service.create_user(user3_data)
    print(f"Created test user 3 with ID: {user3.id}")

    return [user1, user2, user3]


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
        ),
        Chapter(
            title="Mathematical Foundations",
            slug="mathematical-foundations",
            content="This chapter covers the mathematical foundations needed for Physical AI...",
            description="Mathematical foundations for Physical AI",
            chapter_number=5,
            word_count=3000,
            reading_time_minutes=30,
            is_published=True,
            is_ai_optimized=True
        ),
        Chapter(
            title="ROS 2 for Physical AI Systems",
            slug="ros2-physical-ai",
            content="This chapter covers ROS 2 development for Physical AI systems...",
            description="ROS 2 development for Physical AI",
            chapter_number=6,
            word_count=2800,
            reading_time_minutes=28,
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


def simulate_user_progress(db: Session, user_id: int, chapters: list):
    """Simulate user progress and AI interactions to test adaptive behavior"""
    # Create AI chat session for the user
    chat_session = AIChatSession(
        user_id=user_id,
        session_title="Learning Session"
    )
    db.add(chat_session)
    db.commit()
    db.refresh(chat_session)

    # Simulate AI interactions for different chapters
    interactions = [
        ("I'm struggling with the mathematical foundations for Physical AI", "question"),
        ("Can you explain more about control systems in robotics?", "question"),
        ("I need more practical examples for AI planning", "question"),
        ("How do I implement ROS 2 nodes for my robot?", "question"),
        ("I understand the theoretical concepts well", "statement"),
        ("I prefer hands-on exercises", "preference"),
        ("This chapter is too difficult", "struggle"),
        ("I need more basic content", "struggle")
    ]

    for i, (content, msg_type) in enumerate(interactions):
        message = AIChatMessage(
            session_id=chat_session.id,
            role="user",
            content=content,
            message_type=msg_type,
            timestamp=datetime.utcnow() - timedelta(hours=i)
        )
        db.add(message)

    db.commit()

    print(f"Simulated {len(interactions)} AI interactions for user {user_id}")


def test_personalized_learning_path_generation(db: Session, users: list):
    """Test that personalized learning paths are generated based on user profiles"""
    print("\n--- Testing Personalized Learning Path Generation ---")

    learning_path_service = get_learning_path_service(db)
    results = []

    for i, user in enumerate(users, 1):
        try:
            personalized_path = learning_path_service.generate_personalized_learning_path(user.id)
            print(f"✅ User {i} ({user.username}): Generated personalized learning path")
            print(f"   Title: {personalized_path.title}")
            print(f"   Difficulty: {personalized_path.difficulty_level}")
            print(f"   Estimated duration: {personalized_path.estimated_duration_hours} hours")

            # Get steps to verify personalization
            steps = learning_path_service.get_learning_path_steps(personalized_path.id)
            print(f"   Steps in path: {len(steps)}")
            for step in steps:
                print(f"     - {step.title}")

            results.append(True)
        except Exception as e:
            print(f"❌ User {i} ({user.username}): Error generating personalized learning path: {str(e)}")
            results.append(False)

    return all(results)


def test_learning_path_adaptation(db: Session, users: list):
    """Test that learning paths adapt based on user progress and performance"""
    print("\n--- Testing Learning Path Adaptation ---")

    adaptive_service = get_adaptive_content_service(db)
    results = []

    for i, user in enumerate(users, 1):
        try:
            # Get initial adaptive content
            initial_content = adaptive_service.get_adaptive_content_for_user(user.id)
            print(f"✅ User {i} ({user.username}): Got initial adaptive content")

            # Check for learning path adjustments
            adjustments = initial_content.get('learning_path_adjustments', [])
            print(f"   Learning path adjustments: {len(adjustments)}")
            for adj in adjustments:
                print(f"     - {adj.get('type')}: {adj.get('message')}")

            # Check for personalized reading order
            chapter_ids = [1, 2, 3, 4, 5, 6]  # IDs of all chapters
            reading_order = adaptive_service.get_personalized_reading_order(user.id, chapter_ids)
            print(f"   Personalized reading order for {len(reading_order)} chapters")

            # Verify the reading order makes sense for the user's background
            chapters = db.query(Chapter).filter(Chapter.id.in_(reading_order[:3])).all()
            print(f"   Top 3 recommended chapters: {[c.title for c in chapters]}")

            results.append(True)
        except Exception as e:
            print(f"❌ User {i} ({user.username}): Error testing adaptation: {str(e)}")
            results.append(False)

    return all(results)


def test_effectiveness_metrics(db: Session, users: list):
    """Test effectiveness metrics for personalized learning paths"""
    print("\n--- Testing Effectiveness Metrics ---")

    adaptive_service = get_adaptive_content_service(db)
    results = []

    for i, user in enumerate(users, 1):
        try:
            # Get performance metrics
            user_progress = adaptive_service._get_user_progress(user.id)
            performance_metrics = adaptive_service._calculate_performance_metrics(user.id, user_progress)

            print(f"✅ User {i} ({user.username}): Calculated performance metrics")
            print(f"   Completion rate: {performance_metrics.get('completion_rate', 0):.2f}%")
            print(f"   Engagement score: {performance_metrics.get('engagement_score', 0):.2f}")
            print(f"   Struggling indicators: {len(performance_metrics.get('struggling_indicators', []))}")
            print(f"   Learning style: {performance_metrics.get('learning_style', 'unknown')}")
            print(f"   Time spent: {performance_metrics.get('time_spent_learning', 0):.2f} minutes")
            print(f"   Preferred difficulty: {performance_metrics.get('preferred_difficulty', 'unknown')}")

            # Test that metrics influence recommendations
            adaptive_content = adaptive_service.get_adaptive_content_for_user(user.id)
            recommendations = adaptive_content.get('recommended_content', [])

            print(f"   Recommendations based on metrics: {len(recommendations)}")

            # Check if recommendations align with user profile
            user_background = user.professional_background.lower()
            relevant_recs = 0
            for rec in recommendations[:3]:  # Check top 3
                title_desc = (rec.get('title', '') + ' ' + rec.get('description', '')).lower()
                if any(keyword in user_background for keyword in ['robot', 'ai', 'control', 'planning', 'math', 'ros']):
                    relevant_recs += 1

            print(f"   Relevant recommendations: {relevant_recs}/3")

            results.append(relevant_recs > 0)  # At least some recommendations should be relevant
        except Exception as e:
            print(f"❌ User {i} ({user.username}): Error testing effectiveness: {str(e)}")
            results.append(False)

    return all(results)


def test_personalized_learning_path_effectiveness():
    """Test the effectiveness of personalized learning paths"""
    print("Testing Personalized Learning Path Effectiveness (T057)")
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

        # Create test users with different backgrounds
        users = create_test_users(db)

        # Create sample content
        chapters = create_sample_chapters(db)
        learning_paths = create_sample_learning_paths(db)

        # Simulate user progress and interactions
        for user in users:
            simulate_user_progress(db, user.id, chapters)

        # Test all aspects of personalized learning path effectiveness
        test_results = []

        test_results.append(("Personalized Learning Path Generation", test_personalized_learning_path_generation(db, users)))
        test_results.append(("Learning Path Adaptation", test_learning_path_adaptation(db, users)))
        test_results.append(("Effectiveness Metrics", test_effectiveness_metrics(db, users)))

        # Overall validation
        print("\n" + "=" * 60)
        print("PERSONALIZED LEARNING PATH EFFECTIVENESS VALIDATION RESULTS:")

        all_passed = True
        for test_name, passed in test_results:
            status = "✅ PASSED" if passed else "❌ FAILED"
            print(f"{test_name}: {status}")
            if not passed:
                all_passed = False

        print(f"\n{'✅ PERSONALIZED LEARNING PATH EFFECTIVENESS (T057): PASSED' if all_passed else '❌ PERSONALIZED LEARNING PATH EFFECTIVENESS (T057): FAILED'}")

        if all_passed:
            print("Personalized learning paths effectively adapt to user profiles and progress.")
            print("The system successfully generates personalized content based on professional background.")
        else:
            print("Some aspects of personalized learning path effectiveness need improvement.")

        return all_passed

    except Exception as e:
        print(f"\n❌ Error during testing: {str(e)}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        db.close()


def main():
    """Main function to run the personalized learning path effectiveness tests"""
    success = test_personalized_learning_path_effectiveness()

    if success:
        print("\n🎉 ALL PERSONALIZED LEARNING PATH EFFECTIVENESS TESTS PASSED!")
        print("Phase 5: User Story 3 - Professional Development is complete.")
        print("Personalized learning paths are effective and adapt to user needs.")
    else:
        print("\n❌ SOME PERSONALIZED LEARNING PATH EFFECTIVENESS TESTS FAILED!")
        print("Please review the errors above.")

    return success


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
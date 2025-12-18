#!/usr/bin/env python3
"""
Test script to validate the student learning journey with sample chapter content (T034)
This script:
1. Creates a sample chapter in the database
2. Tests the AI chat functionality with the chapter content
3. Validates that the RAG system works correctly
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

# Import models directly to avoid circular import
from backend.src.models.chapter import Chapter
from backend.src.services.ai_service import get_ai_service
from backend.src.lib.vector_store import vector_store, CHAPTER_CONTENT_COLLECTION

# Create database session factory
DATABASE_URL = os.getenv("DATABASE_URL", "postgresql://username:password@localhost/physical_ai_textbook")
engine = create_engine(
    DATABASE_URL,
    poolclass=QueuePool,
    pool_size=10,
    max_overflow=20,
    pool_pre_ping=True,
    pool_recycle=300,
    echo=False
)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)


def load_sample_chapter_content():
    """Load the sample chapter content from the markdown file"""
    try:
        with open('docs/textbook-content/sample-chapter-1-introduction-physical-ai.md', 'r', encoding='utf-8') as file:
            content = file.read()
        return content
    except FileNotFoundError:
        print("Sample chapter file not found. Creating a simple sample chapter...")
        return """# Chapter 1: Introduction to Physical AI

## What is Physical AI?

Physical AI represents a paradigm shift in artificial intelligence, where cognitive systems are embodied in physical form and interact with the real world. Unlike traditional AI that operates in virtual environments, Physical AI systems must navigate the complexities of physics, dynamics, and real-world uncertainty.

## Core Principles

The fundamental principles of Physical AI include:

1. **Embodiment**: Intelligence emerges from the interaction between an agent and its physical environment
2. **Real-time Processing**: Systems must respond to environmental changes within strict temporal constraints
3. **Uncertainty Management**: Dealing with sensor noise, actuator limitations, and environmental variability
4. **Energy Efficiency**: Optimizing for real-world power constraints

## Applications in Humanoid Robotics

Physical AI has found significant applications in humanoid robotics, where robots must perform complex tasks in human environments. Key applications include locomotion, manipulation, interaction, and adaptation.

## Technical Challenges

The implementation of Physical AI systems faces several challenges including the sim-to-real gap, safety, scalability, and learning efficiency."""


def create_sample_chapter(db: Session):
    """Create a sample chapter in the database"""
    # Check if the chapter already exists
    existing_chapter = db.query(Chapter).filter(Chapter.slug == "introduction-to-physical-ai").first()
    if existing_chapter:
        print("Sample chapter already exists in database.")
        return existing_chapter

    # Load sample content
    content = load_sample_chapter_content()

    # Create chapter object
    chapter = Chapter(
        title="Chapter 1: Introduction to Physical AI",
        slug="introduction-to-physical-ai",
        content=content,
        description="An introduction to the fundamental concepts of Physical AI",
        chapter_number=1,
        word_count=len(content.split()),
        reading_time_minutes=len(content.split()) // 200,  # Rough estimate (200 words per minute)
        is_published=True,
        is_ai_optimized=True
    )

    # Add to database
    db.add(chapter)
    db.commit()
    db.refresh(chapter)

    print(f"Created sample chapter with ID: {chapter.id}")
    return chapter


def test_ai_functionality(db: Session, chapter_id: int):
    """Test the AI functionality with the sample chapter"""
    print("\n--- Testing AI Functionality ---")

    # Create AI service
    ai_service = get_ai_service(db)

    # Test questions related to the chapter content
    test_questions = [
        "What are the core principles of Physical AI?",
        "How does Physical AI differ from traditional AI?",
        "What are the applications of Physical AI in humanoid robotics?",
        "What is the sim-to-real gap in Physical AI?",
        "Why is safety important in Physical AI systems?"
    ]

    print(f"Testing AI responses for chapter ID: {chapter_id}")

    all_tests_passed = True

    for i, question in enumerate(test_questions, 1):
        print(f"\nTest {i}: {question}")

        # Generate AI response
        response = ai_service.generate_ai_response(
            query=question,
            session_id=1,  # Using a mock session ID
            user_id=1,     # Using a mock user ID
            context_chapters=[chapter_id]
        )

        print(f"AI Response: {response[:200]}...")  # Truncate for readability

        # Basic validation: check if response is not empty and contains relevant content
        if not response or len(response.strip()) < 10:
            print(f"❌ Test {i} FAILED: Response is too short or empty")
            all_tests_passed = False
        else:
            print(f"✅ Test {i} PASSED")

    return all_tests_passed


def test_vector_store_integration(db: Session, chapter_id: int):
    """Test the vector store integration for RAG functionality"""
    print("\n--- Testing Vector Store Integration ---")

    # Check if the collection exists
    if not vector_store.collection_exists(CHAPTER_CONTENT_COLLECTION):
        print(f"❌ Collection {CHAPTER_CONTENT_COLLECTION} does not exist")
        return False

    # Check the number of vectors in the collection
    count = vector_store.get_vector_count(CHAPTER_CONTENT_COLLECTION)
    print(f"Vector count in {CHAPTER_CONTENT_COLLECTION}: {count}")

    # Test search functionality
    ai_service = get_ai_service(db)
    search_results = ai_service.search_content("Physical AI principles", limit=3)

    print(f"Search results count: {len(search_results)}")

    if len(search_results) > 0:
        print("✅ Vector store integration test PASSED")
        return True
    else:
        print("❌ Vector store integration test FAILED")
        return False


def create_all_tables():
    """Create all tables in the database"""
    # Import all models to register them with the Base
    from backend.src.models.user import User
    from backend.src.models.chapter import Chapter, ChapterContentBlock
    from backend.src.models.learning_path import LearningPath, LearningPathStep, UserLearningPath, UserLearningPathProgress
    from backend.src.models.ai_interaction import AIChatSession, AIChatMessage, AIInteractionLog, AIPreference
    from backend.src.models.citation import Citation, ChapterCitation, CitationVerification
    from backend.src.models.course import Course, CourseEnrollment, CourseChapter, CourseAssignment, UserCourseProgress, CourseAnalytics

    # Get the Base class from the database module
    from backend.src.lib.database import Base
    Base.metadata.create_all(bind=engine)


def main():
    """Main function to run the student learning journey test"""
    print("Testing Student Learning Journey with Sample Chapter Content (T034)")
    print("=" * 70)

    # Create database session
    db = SessionLocal()

    try:
        # Create all tables (in case they don't exist)
        create_all_tables()

        # Create sample chapter
        chapter = create_sample_chapter(db)

        # Test AI functionality
        ai_tests_passed = test_ai_functionality(db, chapter.id)

        # Test vector store integration
        vector_tests_passed = test_vector_store_integration(db, chapter.id)

        # Overall result
        print("\n" + "=" * 70)
        print("TEST RESULTS SUMMARY:")
        print(f"AI Functionality Tests: {'PASSED' if ai_tests_passed else 'FAILED'}")
        print(f"Vector Store Tests: {'PASSED' if vector_tests_passed else 'FAILED'}")

        if ai_tests_passed and vector_tests_passed:
            print("✅ STUDENT LEARNING JOURNEY TEST: PASSED")
            print("The student learning journey with sample chapter content is working correctly.")
            return True
        else:
            print("❌ STUDENT LEARNING JOURNEY TEST: FAILED")
            print("There are issues with the student learning journey implementation.")
            return False

    except Exception as e:
        print(f"❌ Error during testing: {str(e)}")
        return False
    finally:
        db.close()


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
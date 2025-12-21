---
id: 1
title: Fix Circular Imports and Metadata Issues
stage: general
date_iso: 2025-12-18
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: amna_meo
command: sp.implementation
labels: ["bug-fix", "database", "models"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - backend/src/lib/database.py
  - backend/src/models/ai_interaction.py
  - backend/src/models/chapter.py
  - backend/src/models/course.py
  - backend/src/models/citation.py
  - backend/src/models/learning_path.py
  - backend/src/services/ai_service.py
  - backend/migrations/versions/001_initial.py
tests: []
---

# Fix Circular Imports and Metadata Issues

## Prompt Text

sp.implementation

## Response Text

Resolved critical issues in the Physical AI & Humanoid Robotics Textbook project:

1. Fixed circular import issue in database module by moving model imports to a function that's only called when needed
2. Fixed reserved 'metadata' attribute name conflicts in SQLAlchemy models by renaming to more specific names:
   - `metadata` → `session_metadata` (in ai_chat_sessions)
   - `metadata` → `chapter_metadata` (in chapters)
   - `metadata` → `block_metadata` (in chapter_content_blocks)
   - `metadata` → `course_metadata` (in courses)
   - `metadata` → `assignment_metadata` (in course_assignments)
   - `metadata` → `citation_metadata` (in citations)
   - `metadata` → `path_metadata` (in learning_paths)
3. Fixed Pydantic protected namespace conflict with 'model_used' field by renaming to 'ai_model_used'
4. Updated migration files to match new column names
5. Made AI service more robust by handling missing OpenAI API key gracefully
6. Verified that student learning journey functionality works correctly

## Outcome

The implementation successfully resolves the circular import and metadata naming issues that were preventing the application from starting. The AI functionality tests pass, and the system is now ready for the remaining Phase 3 tasks (T034 and T035) to be completed.
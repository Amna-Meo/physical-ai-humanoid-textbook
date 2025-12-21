from typing import Optional, List
from sqlalchemy.orm import Session
from datetime import datetime

from ..models.learning_path import (
    LearningPath, LearningPathCreate, LearningPathUpdate, LearningPathResponse,
    LearningPathStep, LearningPathStepResponse, UserLearningPath, UserLearningPathResponse,
    UserLearningPathProgress
)
from ..models.user import User
from ..models.chapter import Chapter


class LearningPathService:
    def __init__(self, db: Session):
        self.db = db

    def create_learning_path(self, learning_path_create: LearningPathCreate) -> LearningPath:
        """
        Create a new learning path
        """
        db_learning_path = LearningPath(
            title=learning_path_create.title,
            slug=learning_path_create.slug,
            description=learning_path_create.description,
            is_active=learning_path_create.is_active,
            is_personalized=learning_path_create.is_personalized,
            difficulty_level=learning_path_create.difficulty_level,
            estimated_duration_hours=learning_path_create.estimated_duration_hours,
            path_metadata=learning_path_create.path_metadata
        )

        self.db.add(db_learning_path)
        self.db.commit()
        self.db.refresh(db_learning_path)

        return db_learning_path

    def get_learning_path(self, learning_path_id: int) -> Optional[LearningPath]:
        """
        Get a learning path by ID
        """
        return self.db.query(LearningPath).filter(LearningPath.id == learning_path_id).first()

    def get_learning_path_by_slug(self, slug: str) -> Optional[LearningPath]:
        """
        Get a learning path by slug
        """
        return self.db.query(LearningPath).filter(LearningPath.slug == slug).first()

    def get_all_learning_paths(self, skip: int = 0, limit: int = 100) -> List[LearningPath]:
        """
        Get all learning paths with pagination
        """
        return self.db.query(LearningPath).offset(skip).limit(limit).all()

    def update_learning_path(self, learning_path_id: int, learning_path_update: LearningPathUpdate) -> Optional[LearningPath]:
        """
        Update a learning path
        """
        db_learning_path = self.get_learning_path(learning_path_id)
        if not db_learning_path:
            return None

        # Update fields if provided
        if learning_path_update.title is not None:
            db_learning_path.title = learning_path_update.title
        if learning_path_update.description is not None:
            db_learning_path.description = learning_path_update.description
        if learning_path_update.is_active is not None:
            db_learning_path.is_active = learning_path_update.is_active
        if learning_path_update.is_personalized is not None:
            db_learning_path.is_personalized = learning_path_update.is_personalized
        if learning_path_update.difficulty_level is not None:
            db_learning_path.difficulty_level = learning_path_update.difficulty_level
        if learning_path_update.estimated_duration_hours is not None:
            db_learning_path.estimated_duration_hours = learning_path_update.estimated_duration_hours
        if learning_path_update.path_metadata is not None:
            db_learning_path.path_metadata = learning_path_update.path_metadata

        self.db.commit()
        self.db.refresh(db_learning_path)

        return db_learning_path

    def delete_learning_path(self, learning_path_id: int) -> bool:
        """
        Delete a learning path
        """
        db_learning_path = self.get_learning_path(learning_path_id)
        if not db_learning_path:
            return False

        self.db.delete(db_learning_path)
        self.db.commit()

        return True

    def create_learning_path_step(self, learning_path_id: int, step_data: dict) -> LearningPathStep:
        """
        Create a step in a learning path
        """
        # Verify learning path exists
        learning_path = self.get_learning_path(learning_path_id)
        if not learning_path:
            raise ValueError("Learning path not found")

        db_step = LearningPathStep(
            learning_path_id=learning_path_id,
            step_number=step_data.get('step_number'),
            title=step_data.get('title'),
            description=step_data.get('description'),
            content_type=step_data.get('content_type'),
            content_id=step_data.get('content_id'),
            required=step_data.get('required', True),
            estimated_duration_minutes=step_data.get('estimated_duration_minutes')
        )

        self.db.add(db_step)
        self.db.commit()
        self.db.refresh(db_step)

        return db_step

    def get_learning_path_steps(self, learning_path_id: int) -> List[LearningPathStep]:
        """
        Get all steps in a learning path ordered by step number
        """
        return self.db.query(LearningPathStep).filter(
            LearningPathStep.learning_path_id == learning_path_id
        ).order_by(LearningPathStep.step_number).all()

    def get_user_learning_paths(self, user_id: int) -> List[UserLearningPath]:
        """
        Get all learning paths for a user
        """
        return self.db.query(UserLearningPath).filter(
            UserLearningPath.user_id == user_id
        ).all()

    def assign_learning_path_to_user(self, user_id: int, learning_path_id: int) -> UserLearningPath:
        """
        Assign a learning path to a user
        """
        # Verify user exists
        user = self.db.query(User).filter(User.id == user_id).first()
        if not user:
            raise ValueError("User not found")

        # Verify learning path exists
        learning_path = self.get_learning_path(learning_path_id)
        if not learning_path:
            raise ValueError("Learning path not found")

        # Check if user already has this learning path assigned
        existing_assignment = self.db.query(UserLearningPath).filter(
            UserLearningPath.user_id == user_id,
            UserLearningPath.learning_path_id == learning_path_id
        ).first()

        if existing_assignment:
            return existing_assignment

        # Create user learning path assignment
        user_learning_path = UserLearningPath(
            user_id=user_id,
            learning_path_id=learning_path_id,
            status="not_started",
            current_step=0,
            progress_percentage=0
        )

        self.db.add(user_learning_path)
        self.db.commit()
        self.db.refresh(user_learning_path)

        return user_learning_path

    def get_user_learning_path_progress(self, user_id: int, learning_path_id: int) -> Optional[UserLearningPath]:
        """
        Get a specific user's progress on a learning path
        """
        return self.db.query(UserLearningPath).filter(
            UserLearningPath.user_id == user_id,
            UserLearningPath.learning_path_id == learning_path_id
        ).first()

    def update_user_learning_path_progress(self, user_id: int, learning_path_id: int, step_id: int, status: str) -> UserLearningPathProgress:
        """
        Update a user's progress on a specific step in a learning path
        """
        # Verify user exists
        user = self.db.query(User).filter(User.id == user_id).first()
        if not user:
            raise ValueError("User not found")

        # Verify learning path exists
        learning_path = self.get_learning_path(learning_path_id)
        if not learning_path:
            raise ValueError("Learning path not found")

        # Verify step exists
        step = self.db.query(LearningPathStep).filter(LearningPathStep.id == step_id).first()
        if not step:
            raise ValueError("Step not found")

        # Check if progress record already exists
        progress = self.db.query(UserLearningPathProgress).filter(
            UserLearningPathProgress.user_id == user_id,
            UserLearningPathProgress.learning_path_id == learning_path_id,
            UserLearningPathProgress.step_id == step_id
        ).first()

        if progress:
            # Update existing progress
            progress.status = status
            if status == "completed":
                progress.completed_at = datetime.utcnow()
        else:
            # Create new progress record
            progress = UserLearningPathProgress(
                user_id=user_id,
                learning_path_id=learning_path_id,
                step_id=step_id,
                status=status
            )
            self.db.add(progress)

        # Update the overall learning path progress
        user_learning_path = self.get_user_learning_path_progress(user_id, learning_path_id)
        if user_learning_path:
            # Calculate overall progress percentage
            steps = self.get_learning_path_steps(learning_path_id)
            total_steps = len([s for s in steps if s.required])  # Only count required steps
            completed_steps = self.db.query(UserLearningPathProgress).filter(
                UserLearningPathProgress.user_id == user_id,
                UserLearningPathProgress.learning_path_id == learning_path_id,
                UserLearningPathProgress.status == "completed"
            ).count()

            if total_steps > 0:
                progress_percentage = min(100, int((completed_steps / total_steps) * 100))
            else:
                progress_percentage = 0

            user_learning_path.progress_percentage = progress_percentage

            # Update status based on progress
            if progress_percentage == 100:
                user_learning_path.status = "completed"
                user_learning_path.completed_at = datetime.utcnow()
            elif progress_percentage > 0:
                user_learning_path.status = "in_progress"

            # Update current step if needed
            if status == "completed":
                # Find the next incomplete required step
                completed_step_numbers = [
                    s.step_number for s in self.db.query(UserLearningPathProgress)
                    .join(LearningPathStep)
                    .filter(
                        UserLearningPathProgress.user_id == user_id,
                        UserLearningPathProgress.learning_path_id == learning_path_id,
                        UserLearningPathProgress.status == "completed"
                    ).all()
                ]
                next_step_number = max(completed_step_numbers) + 1 if completed_step_numbers else 1
                user_learning_path.current_step = next_step_number

        self.db.commit()
        self.db.refresh(progress)

        return progress

    def generate_personalized_learning_path(self, user_id: int) -> LearningPath:
        """
        Generate a personalized learning path based on user's professional background
        """
        # Get user profile
        user = self.db.query(User).filter(User.id == user_id).first()
        if not user:
            raise ValueError("User not found")

        # Analyze user's professional background and learning preferences
        professional_background = user.professional_background or ""
        learning_preferences = user.learning_preferences or "{}"

        # Based on the user's background, generate an appropriate learning path
        # This is a simplified version - in a real implementation, this would be more sophisticated
        title = f"Personalized Learning Path for {user.full_name or user.username}"
        slug = f"personalized-{user_id}-{int(datetime.utcnow().timestamp())}"

        # Determine difficulty based on background
        difficulty = "beginner"
        if "engineer" in professional_background.lower() or "robotics" in professional_background.lower():
            difficulty = "intermediate"
        elif "phd" in professional_background.lower() or "researcher" in professional_background.lower():
            difficulty = "advanced"

        # Create personalized learning path
        learning_path_create = LearningPathCreate(
            title=title,
            slug=slug,
            description=f"Personalized learning path based on user's professional background: {professional_background}",
            is_active=True,
            is_personalized=True,
            difficulty_level=difficulty,
            estimated_duration_hours=20,  # Default estimate
            path_metadata={
                "generated_from_user_profile": True,
                "user_id": user_id,
                "based_on_professional_background": professional_background,
                "based_on_learning_preferences": learning_preferences
            }
        )

        learning_path = self.create_learning_path(learning_path_create)

        # Add appropriate steps based on user's background
        self._add_steps_based_on_background(learning_path.id, professional_background, difficulty)

        return learning_path

    def _add_steps_based_on_background(self, learning_path_id: int, professional_background: str, difficulty: str):
        """
        Add appropriate steps to the learning path based on user's background
        """
        # Define content based on background
        if "beginner" in difficulty:
            # For beginners, start with fundamentals
            steps = [
                {
                    "step_number": 1,
                    "title": "Introduction to Physical AI",
                    "description": "Understanding the basics of Physical AI and how it differs from traditional AI",
                    "content_type": "chapter",
                    "content_id": 1,  # Assuming chapter 1 is intro
                    "required": True,
                    "estimated_duration_minutes": 60
                },
                {
                    "step_number": 2,
                    "title": "Mathematical Foundations",
                    "description": "Basic mathematical concepts needed for Physical AI",
                    "content_type": "chapter",
                    "content_id": 2,  # Assuming chapter 2 is math
                    "required": True,
                    "estimated_duration_minutes": 90
                }
            ]
        elif "advanced" in difficulty:
            # For advanced users, go deeper more quickly
            steps = [
                {
                    "step_number": 1,
                    "title": "Advanced Control Theory for Physical AI",
                    "description": "Deep dive into control systems for physical systems",
                    "content_type": "chapter",
                    "content_id": 5,  # Assuming chapter 5 is advanced
                    "required": True,
                    "estimated_duration_minutes": 120
                },
                {
                    "step_number": 2,
                    "title": "Real-time Optimization Techniques",
                    "description": "Advanced optimization for real-time physical systems",
                    "content_type": "chapter",
                    "content_id": 6,
                    "required": True,
                    "estimated_duration_minutes": 120
                }
            ]
        else:
            # For intermediate users
            steps = [
                {
                    "step_number": 1,
                    "title": "Physical AI Fundamentals",
                    "description": "Core concepts of Physical AI with practical examples",
                    "content_type": "chapter",
                    "content_id": 3,  # Assuming chapter 3 is intermediate
                    "required": True,
                    "estimated_duration_minutes": 90
                },
                {
                    "step_number": 2,
                    "title": "Control Systems in Physical AI",
                    "description": "Understanding control theory applications",
                    "content_type": "chapter",
                    "content_id": 4,
                    "required": True,
                    "estimated_duration_minutes": 90
                }
            ]

        # Add simulation and practical exercises based on background
        if "robotics" in professional_background.lower():
            # Add more robotics-focused content
            next_step_num = len(steps) + 1
            steps.append({
                "step_number": next_step_num,
                "title": "ROS 2 for Physical AI Systems",
                "description": "Robot Operating System for controlling physical AI systems",
                "content_type": "chapter",
                "content_id": 7,
                "required": True,
                "estimated_duration_minutes": 120
            })
        elif "software" in professional_background.lower():
            # Add more programming-focused content
            next_step_num = len(steps) + 1
            steps.append({
                "step_number": next_step_num,
                "title": "Simulation Environments",
                "description": "Creating and using simulation environments for Physical AI",
                "content_type": "chapter",
                "content_id": 8,
                "required": True,
                "estimated_duration_minutes": 120
            })

        # Add the steps to the learning path
        for step_data in steps:
            self.create_learning_path_step(learning_path_id, step_data)


# Convenience function to get learning path service
def get_learning_path_service(db: Session) -> LearningPathService:
    return LearningPathService(db)
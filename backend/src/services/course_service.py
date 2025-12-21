from typing import Optional, List
from sqlalchemy.orm import Session
from datetime import datetime

from ..models.course import (
    Course, CourseCreate, CourseUpdate, CourseResponse,
    CourseEnrollment, CourseEnrollmentResponse,
    CourseChapter, CourseChapterResponse,
    CourseAssignment, CourseAssignmentUpdate, CourseAssignmentResponse,
    UserCourseProgress, UserCourseProgressResponse
)
from ..models.user import User
from ..models.chapter import Chapter


class CourseService:
    def __init__(self, db: Session):
        self.db = db

    def create_course(self, course_create: CourseCreate) -> Course:
        """
        Create a new course
        """
        # Verify instructor exists
        instructor = self.db.query(User).filter(User.id == course_create.instructor_id).first()
        if not instructor:
            raise ValueError("Instructor not found")

        # Check if course with slug already exists
        existing_course = self.db.query(Course).filter(Course.slug == course_create.slug).first()
        if existing_course:
            raise ValueError("Course with this slug already exists")

        # Create the course
        db_course = Course(
            title=course_create.title,
            slug=course_create.slug,
            description=course_create.description,
            instructor_id=course_create.instructor_id,
            is_active=course_create.is_active,
            is_published=course_create.is_published,
            start_date=course_create.start_date,
            end_date=course_create.end_date,
            enrollment_open=course_create.enrollment_open,
            course_metadata=course_create.course_metadata
        )

        self.db.add(db_course)
        self.db.commit()
        self.db.refresh(db_course)

        return db_course

    def get_course(self, course_id: int) -> Optional[Course]:
        """
        Get a course by ID
        """
        return self.db.query(Course).filter(Course.id == course_id).first()

    def get_course_by_slug(self, slug: str) -> Optional[Course]:
        """
        Get a course by slug
        """
        return self.db.query(Course).filter(Course.slug == slug).first()

    def update_course(self, course_id: int, course_update: CourseUpdate) -> Optional[Course]:
        """
        Update a course
        """
        db_course = self.get_course(course_id)
        if not db_course:
            return None

        # Update fields if provided
        if course_update.title is not None:
            db_course.title = course_update.title
        if course_update.description is not None:
            db_course.description = course_update.description
        if course_update.is_active is not None:
            db_course.is_active = course_update.is_active
        if course_update.is_published is not None:
            db_course.is_published = course_update.is_published
        if course_update.start_date is not None:
            db_course.start_date = course_update.start_date
        if course_update.end_date is not None:
            db_course.end_date = course_update.end_date
        if course_update.enrollment_open is not None:
            db_course.enrollment_open = course_update.enrollment_open
        if course_update.course_metadata is not None:
            db_course.course_metadata = course_update.course_metadata

        self.db.commit()
        self.db.refresh(db_course)

        return db_course

    def get_courses_by_instructor(self, instructor_id: int) -> List[Course]:
        """
        Get all courses by an instructor
        """
        return self.db.query(Course).filter(Course.instructor_id == instructor_id).all()

    def get_published_courses(self) -> List[Course]:
        """
        Get all published courses
        """
        return self.db.query(Course).filter(
            Course.is_published == True,
            Course.is_active == True
        ).all()

    def enroll_user_in_course(self, course_id: int, user_id: int, role: str = "student") -> CourseEnrollment:
        """
        Enroll a user in a course
        """
        # Verify course exists and is active
        course = self.get_course(course_id)
        if not course or not course.is_active:
            raise ValueError("Course not found or inactive")

        if not course.enrollment_open:
            raise ValueError("Enrollment is not open for this course")

        # Verify user exists
        user = self.db.query(User).filter(User.id == user_id).first()
        if not user:
            raise ValueError("User not found")

        # Check if user is already enrolled
        existing_enrollment = self.db.query(CourseEnrollment).filter(
            CourseEnrollment.course_id == course_id,
            CourseEnrollment.user_id == user_id
        ).first()

        if existing_enrollment:
            # Update existing enrollment instead of creating new one
            existing_enrollment.enrollment_status = "active"
            existing_enrollment.role = role
            self.db.commit()
            self.db.refresh(existing_enrollment)
            return existing_enrollment

        # Create new enrollment
        enrollment = CourseEnrollment(
            course_id=course_id,
            user_id=user_id,
            role=role
        )

        self.db.add(enrollment)
        self.db.commit()
        self.db.refresh(enrollment)

        return enrollment

    def get_user_enrollments(self, user_id: int) -> List[CourseEnrollment]:
        """
        Get all courses a user is enrolled in
        """
        return self.db.query(CourseEnrollment).filter(
            CourseEnrollment.user_id == user_id
        ).all()

    def get_course_enrollments(self, course_id: int) -> List[CourseEnrollment]:
        """
        Get all enrollments for a course
        """
        return self.db.query(CourseEnrollment).filter(
            CourseEnrollment.course_id == course_id
        ).all()

    def add_chapter_to_course(self, course_id: int, chapter_id: int, order_index: int,
                             is_required: bool = True, due_date: Optional[datetime] = None) -> CourseChapter:
        """
        Add a chapter to a course
        """
        # Verify course exists
        course = self.get_course(course_id)
        if not course:
            raise ValueError("Course not found")

        # Verify chapter exists
        chapter = self.db.query(Chapter).filter(Chapter.id == chapter_id).first()
        if not chapter:
            raise ValueError("Chapter not found")

        # Check if chapter is already in course
        existing_course_chapter = self.db.query(CourseChapter).filter(
            CourseChapter.course_id == course_id,
            CourseChapter.chapter_id == chapter_id
        ).first()

        if existing_course_chapter:
            raise ValueError("Chapter already added to this course")

        # Create course chapter
        course_chapter = CourseChapter(
            course_id=course_id,
            chapter_id=chapter_id,
            order_index=order_index,
            is_required=is_required,
            due_date=due_date
        )

        self.db.add(course_chapter)
        self.db.commit()
        self.db.refresh(course_chapter)

        return course_chapter

    def get_course_chapters(self, course_id: int) -> List[CourseChapter]:
        """
        Get all chapters in a course ordered by index
        """
        return self.db.query(CourseChapter).filter(
            CourseChapter.course_id == course_id
        ).order_by(CourseChapter.order_index).all()

    def create_assignment(self, assignment_data: CourseAssignment) -> CourseAssignment:
        """
        Create a new assignment for a course
        """
        # Verify course exists
        course = self.get_course(assignment_data.course_id)
        if not course:
            raise ValueError("Course not found")

        # Verify chapter exists if provided
        if assignment_data.chapter_id:
            chapter = self.db.query(Chapter).filter(Chapter.id == assignment_data.chapter_id).first()
            if not chapter:
                raise ValueError("Chapter not found")

        self.db.add(assignment_data)
        self.db.commit()
        self.db.refresh(assignment_data)

        return assignment_data

    def update_assignment(self, assignment_id: int, assignment_update: CourseAssignmentUpdate) -> Optional[CourseAssignment]:
        """
        Update an assignment
        """
        db_assignment = self.db.query(CourseAssignment).filter(CourseAssignment.id == assignment_id).first()
        if not db_assignment:
            return None

        # Update fields if provided
        if assignment_update.title is not None:
            db_assignment.title = assignment_update.title
        if assignment_update.description is not None:
            db_assignment.description = assignment_update.description
        if assignment_update.chapter_id is not None:
            db_assignment.chapter_id = assignment_update.chapter_id
        if assignment_update.assignment_type is not None:
            db_assignment.assignment_type = assignment_update.assignment_type
        if assignment_update.points is not None:
            db_assignment.points = assignment_update.points
        if assignment_update.due_date is not None:
            db_assignment.due_date = assignment_update.due_date
        if assignment_update.is_published is not None:
            db_assignment.is_published = assignment_update.is_published
        if assignment_update.assignment_metadata is not None:
            db_assignment.assignment_metadata = assignment_update.assignment_metadata

        self.db.commit()
        self.db.refresh(db_assignment)

        return db_assignment

    def get_course_assignments(self, course_id: int) -> List[CourseAssignment]:
        """
        Get all assignments for a course
        """
        return self.db.query(CourseAssignment).filter(
            CourseAssignment.course_id == course_id
        ).all()

    def update_user_progress(self, user_id: int, course_id: int, chapter_id: int,
                           progress_percentage: int, score: Optional[int] = None) -> UserCourseProgress:
        """
        Update user progress for a chapter in a course
        """
        # Verify all entities exist
        user = self.db.query(User).filter(User.id == user_id).first()
        if not user:
            raise ValueError("User not found")

        course = self.get_course(course_id)
        if not course:
            raise ValueError("Course not found")

        chapter = self.db.query(Chapter).filter(Chapter.id == chapter_id).first()
        if not chapter:
            raise ValueError("Chapter not found")

        # Check if progress record already exists
        progress = self.db.query(UserCourseProgress).filter(
            UserCourseProgress.user_id == user_id,
            UserCourseProgress.course_id == course_id,
            UserCourseProgress.chapter_id == chapter_id
        ).first()

        if progress:
            # Update existing progress
            progress.progress_percentage = progress_percentage
            if score is not None:
                progress.score = score
            if progress_percentage >= 100 and not progress.completed_at:
                progress.completed_at = datetime.utcnow()
        else:
            # Create new progress record
            progress = UserCourseProgress(
                user_id=user_id,
                course_id=course_id,
                chapter_id=chapter_id,
                progress_percentage=progress_percentage
            )
            if score is not None:
                progress.score = score
            if progress_percentage >= 100:
                progress.completed_at = datetime.utcnow()

            self.db.add(progress)

        self.db.commit()
        self.db.refresh(progress)

        return progress

    def get_user_course_progress(self, user_id: int, course_id: int) -> List[UserCourseProgress]:
        """
        Get all progress records for a user in a course
        """
        return self.db.query(UserCourseProgress).filter(
            UserCourseProgress.user_id == user_id,
            UserCourseProgress.course_id == course_id
        ).all()

    def get_course_completion_rate(self, course_id: int) -> float:
        """
        Calculate the overall completion rate for a course
        """
        # Get all enrolled users in the course
        enrollments = self.get_course_enrollments(course_id)
        if not enrollments:
            return 0.0

        total_users = len(enrollments)
        completed_users = 0

        for enrollment in enrollments:
            # Get all chapters in the course
            course_chapters = self.get_course_chapters(course_id)
            if not course_chapters:
                continue

            # Get user's progress for all chapters in the course
            user_progress = self.get_user_course_progress(enrollment.user_id, course_id)

            # Check if user completed all required chapters
            required_chapters = [cc for cc in course_chapters if cc.is_required]
            if not required_chapters:
                continue

            completed_required_chapters = sum(1 for up in user_progress
                                            if any(cc.chapter_id == up.chapter_id and cc.is_required
                                                   for cc in course_chapters) and up.progress_percentage >= 100)

            if completed_required_chapters >= len(required_chapters):
                completed_users += 1

        return (completed_users / total_users) * 100 if total_users > 0 else 0.0

    def get_course_analytics(self, course_id: int):
        """
        Get comprehensive analytics for a course
        """
        from datetime import datetime, timedelta
        from sqlalchemy import func

        # Get basic course info
        course = self.get_course(course_id)
        if not course:
            return None

        # Get all enrollments for the course
        enrollments = self.get_course_enrollments(course_id)
        total_students = len(enrollments)

        # Calculate course completion rate
        completion_rate = self.get_course_completion_rate(course_id)

        # Calculate average progress across all students
        total_progress = 0
        total_chapters = 0
        for enrollment in enrollments:
            user_progress = self.get_user_course_progress(enrollment.user_id, course_id)
            for progress in user_progress:
                total_progress += progress.progress_percentage
                total_chapters += 1

        avg_progress = (total_progress / total_chapters) if total_chapters > 0 else 0

        # Get all assignments for the course
        assignments = self.get_course_assignments(course_id)
        total_assignments = len(assignments)

        # Calculate assignment statistics
        total_submitted = 0
        total_score = 0
        total_scores_count = 0

        for assignment in assignments:
            # Count submissions and calculate average scores
            # In a real implementation, we might have assignment submission tracking
            # For now, we'll use user progress as a proxy
            for enrollment in enrollments:
                user_progress = self.get_user_course_progress(enrollment.user_id, course_id)
                # Find progress related to the chapter associated with this assignment
                chapter_progress = next((up for up in user_progress if up.chapter_id == assignment.chapter_id), None)
                if chapter_progress and chapter_progress.progress_percentage >= 100:
                    total_submitted += 1
                    if chapter_progress.score is not None:
                        total_score += chapter_progress.score
                        total_scores_count += 1

        avg_assignment_score = (total_score / total_scores_count) if total_scores_count > 0 else 0
        assignment_completion_rate = (total_submitted / (total_assignments * total_students) * 100) if total_assignments > 0 and total_students > 0 else 0

        # Calculate engagement metrics
        active_students = sum(1 for enrollment in enrollments
                             if any(progress.progress_percentage > 0 for progress in self.get_user_course_progress(enrollment.user_id, course_id)))

        engagement_rate = (active_students / total_students * 100) if total_students > 0 else 0

        # Calculate progress by chapter
        course_chapters = self.get_course_chapters(course_id)
        chapter_progress_stats = []
        for course_chapter in course_chapters:
            chapter_progress = self.db.query(UserCourseProgress).filter(
                UserCourseProgress.course_id == course_id,
                UserCourseProgress.chapter_id == course_chapter.chapter_id
            ).all()

            if chapter_progress:
                avg_chapter_progress = sum(p.progress_percentage for p in chapter_progress) / len(chapter_progress)
                completed_count = sum(1 for p in chapter_progress if p.progress_percentage >= 100)
                chapter_stats = {
                    'chapter_id': course_chapter.chapter_id,
                    'order_index': course_chapter.order_index,
                    'avg_progress': round(avg_chapter_progress, 2),
                    'completion_rate': round((completed_count / len(chapter_progress)) * 100, 2),
                    'students_completed': completed_count,
                    'total_students': len(chapter_progress)
                }
                chapter_progress_stats.append(chapter_stats)

        # Calculate recent activity (last 7 days)
        recent_date = datetime.utcnow() - timedelta(days=7)
        recent_progress = self.db.query(UserCourseProgress).filter(
            UserCourseProgress.course_id == course_id,
            UserCourseProgress.updated_at >= recent_date  # assuming there's an updated_at field
        ).count()

        # If updated_at doesn't exist, we'll count all progress records as recent for now
        # In a real implementation, we'd track when progress was last updated
        recent_activity = len([p for p in self.db.query(UserCourseProgress).filter(
            UserCourseProgress.course_id == course_id
        ).all()])

        return {
            'course_id': course_id,
            'course_title': course.title,
            'total_students': total_students,
            'completion_rate': round(completion_rate, 2),
            'avg_progress': round(avg_progress, 2),
            'engagement_rate': round(engagement_rate, 2),
            'total_assignments': total_assignments,
            'assignment_completion_rate': round(assignment_completion_rate, 2),
            'avg_assignment_score': round(avg_assignment_score, 2),
            'chapter_progress_stats': chapter_progress_stats,
            'recent_activity': recent_activity
        }

    def get_instructor_analytics(self, instructor_id: int):
        """
        Get comprehensive analytics across all courses for an instructor
        """
        # Get all courses by the instructor
        courses = self.get_courses_by_instructor(instructor_id)

        total_courses = len(courses)
        total_students_across_courses = 0
        total_completion_rate = 0
        course_analytics_list = []

        for course in courses:
            course_analytics = self.get_course_analytics(course.id)
            if course_analytics:
                course_analytics_list.append(course_analytics)
                total_students_across_courses += course_analytics['total_students']
                total_completion_rate += course_analytics['completion_rate']

        avg_completion_rate = (total_completion_rate / len(course_analytics_list)) if course_analytics_list else 0

        # Calculate overall instructor metrics
        overall_metrics = {
            'instructor_id': instructor_id,
            'total_courses': total_courses,
            'total_students_across_courses': total_students_across_courses,
            'avg_course_completion_rate': round(avg_completion_rate, 2),
            'courses': course_analytics_list
        }

        return overall_metrics


# Convenience function to get course service
def get_course_service(db: Session) -> CourseService:
    return CourseService(db)
from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer
from sqlalchemy.orm import Session
from typing import List, Optional
import os

from ..models.course import (
    Course, CourseResponse, CourseCreate, CourseUpdate,
    CourseEnrollmentResponse,
    CourseChapterResponse,
    CourseAssignmentResponse, CourseAssignmentUpdate,
    UserCourseProgressResponse
)
from ..models.user import User
from ..services.course_service import CourseService, get_course_service
from ..services.user_service import UserService, get_user_service
from ..lib.database import get_db

# OAuth2 scheme for token authentication
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")


def get_current_user_from_token(token: str = Depends(oauth2_scheme), db: Session = Depends(get_db)) -> User:
    """
    Get the current user from the token
    """
    user_service = get_user_service(db)
    payload = user_service.verify_token(token)
    if payload is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

    user_id: int = payload.get("user_id")
    if user_id is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

    user = user_service.get_user_by_id(user_id)
    if user is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="User not found",
            headers={"WWW-Authenticate": "Bearer"},
        )

    if not user.is_active:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Inactive user"
        )

    return user


def check_instructor_access(course_id: int, user: User, db: Session) -> bool:
    """
    Check if the user has instructor access to the specified course
    """
    course_service = get_course_service(db)
    course = course_service.get_course(course_id)

    if not course:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Course not found"
        )

    # Check if the user is the instructor of the course
    if course.instructor_id != user.id:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Access denied: You must be the instructor of this course"
        )

    return True


def check_student_enrollment(course_id: int, user: User, db: Session) -> bool:
    """
    Check if the user is enrolled in the specified course
    """
    course_service = get_course_service(db)

    # Check if user is enrolled in the course
    enrollments = course_service.get_course_enrollments(course_id)
    user_enrolled = any(enrollment.user_id == user.id for enrollment in enrollments)

    # Also allow if the user is the instructor
    course = course_service.get_course(course_id)
    is_instructor = course and course.instructor_id == user.id

    if not user_enrolled and not is_instructor:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Access denied: You must be enrolled in this course or be the instructor"
        )

    return True


# Create router
router = APIRouter(
    prefix="/courses",
    tags=["courses"],
    responses={404: {"description": "Not found"}}
)


@router.get("/", response_model=List[CourseResponse])
async def get_courses(
    skip: int = 0,
    limit: int = 100,
    published_only: bool = True,
    current_user: User = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Get a list of courses
    """
    course_service = get_course_service(db)

    if published_only:
        courses = course_service.get_published_courses()
    else:
        # If not published_only, return all courses the user has access to
        # This could be their enrolled courses, courses they teach, etc.
        # For now, return published courses for non-instructors
        courses = course_service.get_published_courses()

    return courses[skip:skip+limit]


@router.get("/instructor/{instructor_id}", response_model=List[CourseResponse])
async def get_courses_by_instructor(
    instructor_id: int,
    current_user: User = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Get courses by instructor ID
    """
    course_service = get_course_service(db)

    # Only allow the instructor to view their own courses, or allow if the requesting user is the same as instructor_id
    if current_user.id != instructor_id:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Access denied: You can only view your own courses"
        )

    courses = course_service.get_courses_by_instructor(instructor_id)
    return courses


@router.get("/{course_id}", response_model=CourseResponse)
async def get_course(
    course_id: int,
    current_user: User = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Get a specific course by ID
    """
    course_service = get_course_service(db)
    course = course_service.get_course(course_id)

    if not course:
        raise HTTPException(status_code=404, detail="Course not found")

    # Check if user has permission to view this course
    # Allow if course is published and public, or if user is enrolled, or if user is the instructor
    if not course.is_published:
        # For unpublished courses, only the instructor can access
        if course.instructor_id != current_user.id:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Access denied: Course not published or you don't have access"
            )

    return course


@router.post("/", response_model=CourseResponse)
async def create_course(
    course: CourseCreate,
    current_user: User = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Create a new course
    """
    # Ensure the instructor creating the course is the same as the one specified in the request
    if course.instructor_id != current_user.id:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Access denied: You can only create courses for yourself"
        )

    course_service = get_course_service(db)

    try:
        new_course = course_service.create_course(course)
        return new_course
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.put("/{course_id}", response_model=CourseResponse)
async def update_course(
    course_id: int,
    course_update: CourseUpdate,
    current_user: User = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Update a course
    """
    course_service = get_course_service(db)

    # Check if the current user has instructor access to this course
    course = course_service.get_course(course_id)
    if not course:
        raise HTTPException(status_code=404, detail="Course not found")

    if course.instructor_id != current_user.id:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Access denied: You must be the instructor of this course to update it"
        )

    updated_course = course_service.update_course(course_id, course_update)

    if not updated_course:
        raise HTTPException(status_code=404, detail="Course not found")

    return updated_course


@router.post("/{course_id}/enroll", response_model=CourseEnrollmentResponse)
async def enroll_in_course(
    course_id: int,
    user_id: int,
    role: str = "student",
    current_user: User = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Enroll a user in a course
    """
    course_service = get_course_service(db)

    # Check permissions: users can enroll themselves, or instructors can enroll others in their course
    if current_user.id != user_id:
        # If not enrolling themselves, check if they're the instructor
        course = course_service.get_course(course_id)
        if not course or course.instructor_id != current_user.id:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Access denied: You can only enroll yourself or you're not the instructor of this course"
            )

    try:
        enrollment = course_service.enroll_user_in_course(course_id, user_id, role)
        return enrollment
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.get("/{course_id}/enrollments", response_model=List[CourseEnrollmentResponse])
async def get_course_enrollments(
    course_id: int,
    current_user: User = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Get all enrollments for a course
    """
    course_service = get_course_service(db)

    # Only allow the instructor of the course to view enrollments
    course = course_service.get_course(course_id)
    if not course or course.instructor_id != current_user.id:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Access denied: You must be the instructor of this course to view enrollments"
        )

    enrollments = course_service.get_course_enrollments(course_id)
    return enrollments


@router.get("/user/{user_id}/enrollments", response_model=List[CourseEnrollmentResponse])
async def get_user_enrollments(
    user_id: int,
    current_user: User = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Get all courses a user is enrolled in
    """
    course_service = get_course_service(db)

    # Only allow users to view their own enrollments, or instructors to view their students' enrollments
    if current_user.id != user_id:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Access denied: You can only view your own enrollments"
        )

    enrollments = course_service.get_user_enrollments(user_id)
    return enrollments


@router.post("/{course_id}/chapters", response_model=CourseChapterResponse)
async def add_chapter_to_course(
    course_id: int,
    chapter_id: int,
    order_index: int,
    is_required: bool = True,
    due_date: Optional[str] = None,
    current_user: User = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Add a chapter to a course
    """
    from datetime import datetime
    due_date_obj = None
    if due_date:
        try:
            due_date_obj = datetime.fromisoformat(due_date.replace('Z', '+00:00'))
        except ValueError:
            raise HTTPException(status_code=400, detail="Invalid date format")

    course_service = get_course_service(db)

    # Check if the current user has instructor access to this course
    course = course_service.get_course(course_id)
    if not course or course.instructor_id != current_user.id:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Access denied: You must be the instructor of this course to add chapters"
        )

    try:
        course_chapter = course_service.add_chapter_to_course(
            course_id, chapter_id, order_index, is_required, due_date_obj
        )
        return course_chapter
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.get("/{course_id}/chapters", response_model=List[CourseChapterResponse])
async def get_course_chapters(
    course_id: int,
    current_user: User = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Get all chapters in a course
    """
    course_service = get_course_service(db)

    # Check if user has permission to view this course's chapters
    # Allow if course is published and public, or if user is enrolled, or if user is the instructor
    course = course_service.get_course(course_id)
    if not course:
        raise HTTPException(status_code=404, detail="Course not found")

    if not course.is_published:
        # For unpublished courses, only the instructor can access
        if course.instructor_id != current_user.id:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Access denied: Course not published or you don't have access"
            )

    course_chapters = course_service.get_course_chapters(course_id)
    return course_chapters


@router.post("/{course_id}/assignments", response_model=CourseAssignmentResponse)
async def create_assignment(
    course_id: int,
    title: str,
    description: Optional[str] = None,
    chapter_id: Optional[int] = None,
    assignment_type: str = "reading",
    points: int = 100,
    due_date: Optional[str] = None,
    is_published: bool = False,
    current_user: User = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Create a new assignment for a course
    """
    from datetime import datetime
    due_date_obj = None
    if due_date:
        try:
            due_date_obj = datetime.fromisoformat(due_date.replace('Z', '+00:00'))
        except ValueError:
            raise HTTPException(status_code=400, detail="Invalid date format")

    course_service = get_course_service(db)

    # Check if the current user has instructor access to this course
    course = course_service.get_course(course_id)
    if not course or course.instructor_id != current_user.id:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Access denied: You must be the instructor of this course to create assignments"
        )

    # Create assignment data
    from ..models.course import CourseAssignment
    assignment_data = CourseAssignment(
        course_id=course_id,
        title=title,
        description=description,
        chapter_id=chapter_id,
        assignment_type=assignment_type,
        points=points,
        due_date=due_date_obj,
        is_published=is_published
    )

    try:
        assignment = course_service.create_assignment(assignment_data)
        return assignment
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.put("/{course_id}/assignments/{assignment_id}", response_model=CourseAssignmentResponse)
async def update_assignment(
    course_id: int,
    assignment_id: int,
    assignment_update: CourseAssignmentUpdate,
    current_user: User = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Update an assignment
    """
    course_service = get_course_service(db)

    # Check if the current user has instructor access to this course
    course = course_service.get_course(course_id)
    if not course or course.instructor_id != current_user.id:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Access denied: You must be the instructor of this course to update assignments"
        )

    updated_assignment = course_service.update_assignment(assignment_id, assignment_update)

    if not updated_assignment:
        raise HTTPException(status_code=404, detail="Assignment not found")

    return updated_assignment


@router.get("/{course_id}/assignments", response_model=List[CourseAssignmentResponse])
async def get_course_assignments(
    course_id: int,
    current_user: User = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Get all assignments for a course
    """
    course_service = get_course_service(db)

    # Check if user has permission to view this course's assignments
    # Allow if course is published and public, or if user is enrolled, or if user is the instructor
    course = course_service.get_course(course_id)
    if not course:
        raise HTTPException(status_code=404, detail="Course not found")

    if not course.is_published:
        # For unpublished courses, only the instructor can access
        if course.instructor_id != current_user.id:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Access denied: Course not published or you don't have access"
            )

    assignments = course_service.get_course_assignments(course_id)
    return assignments


@router.post("/{course_id}/progress", response_model=UserCourseProgressResponse)
async def update_user_progress(
    course_id: int,
    user_id: int,
    chapter_id: int,
    progress_percentage: int,
    score: Optional[int] = None,
    current_user: User = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Update user progress for a chapter in a course
    """
    course_service = get_course_service(db)

    # Check permissions: users can update their own progress, or instructors can update for their course
    if current_user.id != user_id:
        # If not updating their own progress, check if they're the instructor
        course = course_service.get_course(course_id)
        if not course or course.instructor_id != current_user.id:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Access denied: You can only update your own progress or you're not the instructor of this course"
            )

    try:
        progress = course_service.update_user_progress(
            user_id, course_id, chapter_id, progress_percentage, score
        )
        return progress
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.get("/{course_id}/progress/{user_id}", response_model=List[UserCourseProgressResponse])
async def get_user_course_progress(
    course_id: int,
    user_id: int,
    current_user: User = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Get all progress records for a user in a course
    """
    course_service = get_course_service(db)

    # Only allow users to view their own progress, or instructors to view student progress in their course
    if current_user.id != user_id:
        # Check if the current user is the instructor of the course
        course = course_service.get_course(course_id)
        if not course or course.instructor_id != current_user.id:
            raise HTTPException(
                status_code=status.HTTP_403_FORBIDDEN,
                detail="Access denied: You can only view your own progress or you're not the instructor of this course"
            )

    progress = course_service.get_user_course_progress(user_id, course_id)
    return progress


@router.get("/{course_id}/analytics/completion-rate")
async def get_course_completion_rate(
    course_id: int,
    current_user: User = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Get the completion rate for a course
    """
    course_service = get_course_service(db)

    # Only allow the instructor of the course to access analytics
    course = course_service.get_course(course_id)
    if not course or course.instructor_id != current_user.id:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Access denied: You must be the instructor of this course to access analytics"
        )

    completion_rate = course_service.get_course_completion_rate(course_id)

    return {"course_id": course_id, "completion_rate": completion_rate}


@router.get("/{course_id}/analytics")
async def get_course_analytics(
    course_id: int,
    current_user: User = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Get comprehensive analytics for a course
    """
    course_service = get_course_service(db)

    # Only allow the instructor of the course to access analytics
    course = course_service.get_course(course_id)
    if not course or course.instructor_id != current_user.id:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Access denied: You must be the instructor of this course to access analytics"
        )

    analytics = course_service.get_course_analytics(course_id)

    if not analytics:
        raise HTTPException(status_code=404, detail="Course not found")

    return analytics


@router.get("/instructor/{instructor_id}/analytics")
async def get_instructor_analytics(
    instructor_id: int,
    current_user: User = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Get comprehensive analytics across all courses for an instructor
    """
    # Only allow the instructor to access their own analytics
    if current_user.id != instructor_id:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Access denied: You can only access your own analytics"
        )

    course_service = get_course_service(db)
    analytics = course_service.get_instructor_analytics(instructor_id)

    return analytics


# Include this router in the main app
# This would be done in main.py: app.include_router(course_routes.router, prefix="/api/v1/courses", tags=["courses"])
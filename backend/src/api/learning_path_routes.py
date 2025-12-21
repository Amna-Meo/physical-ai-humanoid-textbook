from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import List, Optional

from ..models.learning_path import (
    LearningPathResponse, LearningPathCreate, LearningPathUpdate,
    LearningPathStepResponse, UserLearningPathResponse
)
from ..models.user import User
from ..services.learning_path_service import get_learning_path_service
from ..services.user_service import get_user_service
from ..services.content_recommendation_service import get_content_recommendation_service
from ..services.adaptive_content_service import get_adaptive_content_service
from ..lib.database import get_db
from .user_routes import get_current_user_from_token

# Create router
router = APIRouter(
    prefix="/learning-paths",
    tags=["learning-paths"],
    responses={404: {"description": "Not found"}}
)


@router.get("/", response_model=List[LearningPathResponse])
async def get_learning_paths(
    skip: int = 0,
    limit: int = 100,
    current_user: User = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Get all learning paths
    """
    learning_path_service = get_learning_path_service(db)
    learning_paths = learning_path_service.get_all_learning_paths(skip=skip, limit=limit)
    return learning_paths


@router.get("/{learning_path_id}", response_model=LearningPathResponse)
async def get_learning_path(
    learning_path_id: int,
    current_user: User = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Get a specific learning path by ID
    """
    learning_path_service = get_learning_path_service(db)
    learning_path = learning_path_service.get_learning_path(learning_path_id)

    if not learning_path:
        raise HTTPException(status_code=404, detail="Learning path not found")

    return learning_path


@router.post("/", response_model=LearningPathResponse)
async def create_learning_path(
    learning_path: LearningPathCreate,
    current_user: User = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Create a new learning path
    """
    learning_path_service = get_learning_path_service(db)

    # Check if user has permission to create learning paths
    # For now, only allow admin users or instructors
    user_service = get_user_service(db)
    # In a real implementation, we'd check user roles

    try:
        new_learning_path = learning_path_service.create_learning_path(learning_path)
        return new_learning_path
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.put("/{learning_path_id}", response_model=LearningPathResponse)
async def update_learning_path(
    learning_path_id: int,
    learning_path_update: LearningPathUpdate,
    current_user: User = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Update a learning path
    """
    learning_path_service = get_learning_path_service(db)

    updated_learning_path = learning_path_service.update_learning_path(learning_path_id, learning_path_update)

    if not updated_learning_path:
        raise HTTPException(status_code=404, detail="Learning path not found")

    return updated_learning_path


@router.delete("/{learning_path_id}")
async def delete_learning_path(
    learning_path_id: int,
    current_user: User = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Delete a learning path
    """
    learning_path_service = get_learning_path_service(db)

    success = learning_path_service.delete_learning_path(learning_path_id)

    if not success:
        raise HTTPException(status_code=404, detail="Learning path not found")

    return {"message": "Learning path deleted successfully"}


@router.get("/{learning_path_id}/steps", response_model=List[LearningPathStepResponse])
async def get_learning_path_steps(
    learning_path_id: int,
    current_user: User = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Get all steps in a learning path
    """
    learning_path_service = get_learning_path_service(db)

    # Verify learning path exists
    learning_path = learning_path_service.get_learning_path(learning_path_id)
    if not learning_path:
        raise HTTPException(status_code=404, detail="Learning path not found")

    steps = learning_path_service.get_learning_path_steps(learning_path_id)
    return steps


@router.get("/user/{user_id}", response_model=List[UserLearningPathResponse])
async def get_user_learning_paths(
    user_id: int,
    current_user: User = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Get all learning paths assigned to a user
    """
    # Check if current user is requesting their own data or is an admin
    if current_user.id != user_id:
        raise HTTPException(status_code=403, detail="Access denied: You can only view your own learning paths")

    learning_path_service = get_learning_path_service(db)
    user_learning_paths = learning_path_service.get_user_learning_paths(user_id)
    return user_learning_paths


@router.post("/assign/{user_id}/{learning_path_id}", response_model=UserLearningPathResponse)
async def assign_learning_path_to_user(
    user_id: int,
    learning_path_id: int,
    current_user: User = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Assign a learning path to a user
    """
    learning_path_service = get_learning_path_service(db)

    # Check permissions - users can assign to themselves, or admins can assign
    if current_user.id != user_id:
        # In a real implementation, we'd check for admin privileges
        raise HTTPException(status_code=403, detail="Access denied: You can only assign learning paths to yourself")

    try:
        user_learning_path = learning_path_service.assign_learning_path_to_user(user_id, learning_path_id)
        return user_learning_path
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.post("/generate-personalized/{user_id}", response_model=LearningPathResponse)
async def generate_personalized_learning_path(
    user_id: int,
    current_user: User = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Generate a personalized learning path based on user's professional background
    """
    # Check permissions - users can only generate for themselves
    if current_user.id != user_id:
        raise HTTPException(status_code=403, detail="Access denied: You can only generate personalized paths for yourself")

    learning_path_service = get_learning_path_service(db)

    try:
        personalized_path = learning_path_service.generate_personalized_learning_path(user_id)
        return personalized_path
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.get("/recommendations/content/{user_id}")
async def get_content_recommendations(
    user_id: int,
    limit: int = 10,
    current_user: User = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Get content recommendations based on user profile
    """
    # Check permissions - users can only get recommendations for themselves
    if current_user.id != user_id:
        raise HTTPException(status_code=403, detail="Access denied: You can only get recommendations for yourself")

    recommendation_service = get_content_recommendation_service(db)

    try:
        recommendations = recommendation_service.get_content_recommendations(user_id, limit=limit)
        return {"recommendations": recommendations}
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.get("/recommendations/learning-paths/{user_id}")
async def get_learning_path_recommendations(
    user_id: int,
    limit: int = 5,
    current_user: User = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Get learning path recommendations based on user profile
    """
    # Check permissions - users can only get recommendations for themselves
    if current_user.id != user_id:
        raise HTTPException(status_code=403, detail="Access denied: You can only get recommendations for yourself")

    recommendation_service = get_content_recommendation_service(db)

    try:
        recommendations = recommendation_service.get_learning_path_recommendations(user_id, limit=limit)
        return {"recommendations": recommendations}
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.get("/adaptive-content/{user_id}")
async def get_adaptive_content(
    user_id: int,
    current_chapter_id: Optional[int] = None,
    current_user: User = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Get adaptive content recommendations based on user's progress and performance
    """
    # Check permissions - users can only get adaptive content for themselves
    if current_user.id != user_id:
        raise HTTPException(status_code=403, detail="Access denied: You can only get adaptive content for yourself")

    adaptive_service = get_adaptive_content_service(db)

    try:
        adaptive_content = adaptive_service.get_adaptive_content_for_user(user_id, current_chapter_id)
        return adaptive_content
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.post("/personalized-reading-order/{user_id}")
async def get_personalized_reading_order(
    user_id: int,
    chapter_ids: List[int],
    current_user: User = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Get a personalized reading order for specified chapters based on user profile
    """
    # Check permissions - users can only get reading order for themselves
    if current_user.id != user_id:
        raise HTTPException(status_code=403, detail="Access denied: You can only get reading order for yourself")

    adaptive_service = get_adaptive_content_service(db)

    try:
        reading_order = adaptive_service.get_personalized_reading_order(user_id, chapter_ids)
        return {"reading_order": reading_order}
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
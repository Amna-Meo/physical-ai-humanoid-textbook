from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer
from sqlalchemy.orm import Session
from typing import List, Optional
import os

from ..models.chapter import (
    Chapter, ChapterResponse, ChapterCreate, ChapterUpdate,
    ChapterContentBlock, ChapterContentBlockResponse
)
from ..models.user import User
from ..services.ai_service import AIService, get_ai_service
from ..lib.database import get_db

# OAuth2 scheme for token authentication
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")

# Create router
router = APIRouter(
    prefix="/chapters",
    tags=["chapters"],
    responses={404: {"description": "Not found"}}
)


@router.get("/", response_model=List[ChapterResponse])
async def get_chapters(
    skip: int = 0,
    limit: int = 100,
    search: Optional[str] = None,
    token: str = Depends(oauth2_scheme),
    db: Session = Depends(get_db)
):
    """
    Get a list of chapters with optional search
    """
    # Verify token (basic check - in a real app, use the user service to extract user info)
    if not token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not authenticated",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Build query
    query = db.query(Chapter)

    # Apply search filter if provided
    if search:
        query = query.filter(Chapter.title.contains(search) | Chapter.content.contains(search))

    # Only return published chapters
    query = query.filter(Chapter.is_published == True)

    # Apply pagination
    chapters = query.offset(skip).limit(limit).all()

    return chapters


@router.get("/{chapter_id}", response_model=ChapterResponse)
async def get_chapter(
    chapter_id: int,
    token: str = Depends(oauth2_scheme),
    db: Session = Depends(get_db)
):
    """
    Get a specific chapter by ID
    """
    # Verify token
    if not token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not authenticated",
            headers={"WWW-Authenticate": "Bearer"},
        )

    chapter = db.query(Chapter).filter(Chapter.id == chapter_id).first()
    if not chapter:
        raise HTTPException(status_code=404, detail="Chapter not found")

    if not chapter.is_published:
        raise HTTPException(status_code=404, detail="Chapter not found")

    return chapter


@router.post("/", response_model=ChapterResponse)
async def create_chapter(
    chapter: ChapterCreate,
    token: str = Depends(oauth2_scheme),  # Would require admin privileges in real implementation
    db: Session = Depends(get_db)
):
    """
    Create a new chapter (admin only in real implementation)
    """
    # Verify token
    if not token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not authenticated",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Check if chapter with slug already exists
    existing_chapter = db.query(Chapter).filter(Chapter.slug == chapter.slug).first()
    if existing_chapter:
        raise HTTPException(status_code=400, detail="Chapter with this slug already exists")

    # Create the chapter
    db_chapter = Chapter(
        title=chapter.title,
        slug=chapter.slug,
        content=chapter.content,
        description=chapter.description,
        chapter_number=chapter.chapter_number,
        is_published=chapter.is_published
    )

    db.add(db_chapter)
    db.commit()
    db.refresh(db_chapter)

    # Add content to vector store for RAG
    ai_service = get_ai_service(db)
    ai_service.add_content_to_vector_store(
        chapter_id=db_chapter.id,
        content=db_chapter.content,
        section="full_content"
    )

    return db_chapter


@router.put("/{chapter_id}", response_model=ChapterResponse)
async def update_chapter(
    chapter_id: int,
    chapter_update: ChapterUpdate,
    token: str = Depends(oauth2_scheme),  # Would require admin privileges in real implementation
    db: Session = Depends(get_db)
):
    """
    Update a chapter (admin only in real implementation)
    """
    # Verify token
    if not token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not authenticated",
            headers={"WWW-Authenticate": "Bearer"},
        )

    db_chapter = db.query(Chapter).filter(Chapter.id == chapter_id).first()
    if not db_chapter:
        raise HTTPException(status_code=404, detail="Chapter not found")

    # Update fields if provided
    if chapter_update.title is not None:
        db_chapter.title = chapter_update.title
    if chapter_update.content is not None:
        db_chapter.content = chapter_update.content
    if chapter_update.description is not None:
        db_chapter.description = chapter_update.description
    if chapter_update.chapter_number is not None:
        db_chapter.chapter_number = chapter_update.chapter_number
    if chapter_update.is_published is not None:
        db_chapter.is_published = chapter_update.is_published

    db.commit()
    db.refresh(db_chapter)

    # Update content in vector store if content changed
    if chapter_update.content is not None:
        ai_service = get_ai_service(db)
        ai_service.add_content_to_vector_store(
            chapter_id=db_chapter.id,
            content=db_chapter.content,
            section="full_content"
        )

    return db_chapter


@router.get("/{chapter_id}/content-blocks", response_model=List[ChapterContentBlockResponse])
async def get_chapter_content_blocks(
    chapter_id: int,
    token: str = Depends(oauth2_scheme),
    db: Session = Depends(get_db)
):
    """
    Get content blocks for a chapter
    """
    # Verify token
    if not token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not authenticated",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Check if chapter exists and is published
    chapter = db.query(Chapter).filter(Chapter.id == chapter_id).first()
    if not chapter or not chapter.is_published:
        raise HTTPException(status_code=404, detail="Chapter not found")

    content_blocks = db.query(ChapterContentBlock).filter(
        ChapterContentBlock.chapter_id == chapter_id
    ).order_by(ChapterContentBlock.order_index).all()

    return content_blocks


@router.get("/{chapter_id}/search", response_model=List[dict])
async def search_in_chapter(
    chapter_id: int,
    query: str,
    limit: int = 10,
    token: str = Depends(oauth2_scheme),
    db: Session = Depends(get_db)
):
    """
    Search within a specific chapter using vector search
    """
    # Verify token
    if not token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not authenticated",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Check if chapter exists and is published
    chapter = db.query(Chapter).filter(Chapter.id == chapter_id).first()
    if not chapter or not chapter.is_published:
        raise HTTPException(status_code=404, detail="Chapter not found")

    # Use AI service to search within the chapter
    ai_service = get_ai_service(db)

    # For chapter-specific search, we'll add a filter for the chapter ID
    results = ai_service.search_content(
        query=query,
        limit=limit,
        filters={"chapter_id": chapter_id}  # This would work in a real implementation
    )

    # Format results
    formatted_results = []
    for result in results:
        formatted_results.append({
            "id": result.get("id"),
            "content": result.get("payload", {}).get("content", "")[:200] + "...",  # Truncate for display
            "score": result.get("score"),
            "section": result.get("payload", {}).get("section", ""),
            "chapter_id": result.get("payload", {}).get("chapter_id")
        })

    return formatted_results


# Include this router in the main app
# This would be done in main.py: app.include_router(chapter_routes.router, prefix="/api/v1/chapters", tags=["chapters"])
from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer
from sqlalchemy.orm import Session
from typing import Optional, List
import os

from ..models.ai_interaction import (
    AIChatSession, AIChatMessage, AIChatSessionResponse,
    AIChatMessageResponse, AIChatMessageCreate, AIChatSessionCreate
)
from ..models.chapter import ChapterResponse
from ..services.ai_service import AIService, get_ai_service
from ..services.error_handling_service import ErrorHandlingService, get_error_handling_service, ErrorCategory
from ..lib.database import get_db

# OAuth2 scheme for token authentication
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")

# Create router
router = APIRouter(
    prefix="/ai",
    tags=["ai"],
    responses={404: {"description": "Not found"}}
)


@router.post("/chat-sessions", response_model=AIChatSessionResponse)
async def create_chat_session(
    session_data: AIChatSessionCreate,
    token: str = Depends(oauth2_scheme),
    db: Session = Depends(get_db)
):
    """
    Create a new AI chat session
    """
    try:
        # Verify token (basic check - in a real app, use the user service)
        if not token:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Not authenticated",
                headers={"WWW-Authenticate": "Bearer"},
            )

        ai_service = get_ai_service(db)
        session = ai_service.create_chat_session(
            user_id=None,  # Would extract from token in a real implementation
            session_title=session_data.session_title
        )
        return session
    except HTTPException:
        # Re-raise HTTP exceptions as they are properly handled
        raise
    except Exception as e:
        # Use error handling service for other exceptions
        error_service = get_error_handling_service(db)
        error_response = error_service.handle_api_error(
            e,
            context={"endpoint": "/ai/chat-sessions", "action": "create_session"},
            severity=error_service.ErrorSeverity.MEDIUM
        )
        raise HTTPException(
            status_code=500,
            detail=error_response["message"]
        )


@router.get("/chat-sessions/{session_id}", response_model=AIChatSessionResponse)
async def get_chat_session(
    session_id: int,
    token: str = Depends(oauth2_scheme),
    db: Session = Depends(get_db)
):
    """
    Get an AI chat session
    """
    try:
        # Verify token
        if not token:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Not authenticated",
                headers={"WWW-Authenticate": "Bearer"},
            )

        ai_service = get_ai_service(db)
        session = ai_service.get_chat_session(session_id)
        if not session:
            raise HTTPException(
                status_code=404,
                detail="Chat session not found"
            )

        return session
    except HTTPException:
        # Re-raise HTTP exceptions as they are properly handled
        raise
    except Exception as e:
        # Use error handling service for other exceptions
        error_service = get_error_handling_service(db)
        error_response = error_service.handle_api_error(
            e,
            context={"endpoint": f"/ai/chat-sessions/{session_id}", "action": "get_session", "session_id": session_id},
            severity=error_service.ErrorSeverity.MEDIUM
        )
        raise HTTPException(
            status_code=500,
            detail=error_response["message"]
        )


@router.post("/chat-sessions/{session_id}/messages", response_model=AIChatMessageResponse)
async def send_message(
    session_id: int,
    message_data: AIChatMessageCreate,
    token: str = Depends(oauth2_scheme),
    db: Session = Depends(get_db)
):
    """
    Send a message to the AI and get a response
    """
    try:
        # Verify token
        if not token:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Not authenticated",
                headers={"WWW-Authenticate": "Bearer"},
            )

        ai_service = get_ai_service(db)

        # Add user message to session
        user_message = ai_service.add_message_to_session(
            session_id=session_id,
            user_id=None,  # Would extract from token in a real implementation
            role=message_data.role,
            content=message_data.content
        )

        # Generate AI response based on the user message and any context
        ai_response = ai_service.generate_ai_response(
            query=message_data.content,
            session_id=session_id,
            context_chapters=message_data.context_chapters
        )

        # Add AI response to session
        ai_message = ai_service.add_message_to_session(
            session_id=session_id,
            user_id=None,
            role="assistant",
            content=ai_response
        )

        return ai_message
    except HTTPException:
        # Re-raise HTTP exceptions as they are properly handled
        raise
    except Exception as e:
        # Use error handling service for other exceptions
        error_service = get_error_handling_service(db)
        error_response = error_service.handle_api_error(
            e,
            context={
                "endpoint": f"/ai/chat-sessions/{session_id}/messages",
                "action": "send_message",
                "session_id": session_id
            },
            severity=error_service.ErrorSeverity.HIGH
        )
        raise HTTPException(
            status_code=500,
            detail=error_response["message"]
        )


@router.get("/chat-sessions/{session_id}/messages", response_model=List[AIChatMessageResponse])
async def get_session_messages(
    session_id: int,
    skip: int = 0,
    limit: int = 50,
    token: str = Depends(oauth2_scheme),
    db: Session = Depends(get_db)
):
    """
    Get messages from a chat session
    """
    try:
        # Verify token
        if not token:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Not authenticated",
                headers={"WWW-Authenticate": "Bearer"},
            )

        ai_service = get_ai_service(db)
        messages = ai_service.get_session_history(session_id, limit=limit)

        # Reverse to return in chronological order (oldest first)
        return list(reversed(messages))
    except HTTPException:
        # Re-raise HTTP exceptions as they are properly handled
        raise
    except Exception as e:
        # Use error handling service for other exceptions
        error_service = get_error_handling_service(db)
        error_response = error_service.handle_api_error(
            e,
            context={
                "endpoint": f"/ai/chat-sessions/{session_id}/messages",
                "action": "get_session_messages",
                "session_id": session_id,
                "skip": skip,
                "limit": limit
            },
            severity=error_service.ErrorSeverity.MEDIUM
        )
        raise HTTPException(
            status_code=500,
            detail=error_response["message"]
        )


@router.get("/search", response_model=List[dict])
async def search_content(
    query: str,
    limit: int = 5,
    token: str = Depends(oauth2_scheme),
    db: Session = Depends(get_db)
):
    """
    Search for content in the textbook using vector search
    """
    # Verify token
    if not token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not authenticated",
            headers={"WWW-Authenticate": "Bearer"},
        )

    ai_service = get_ai_service(db)
    results = ai_service.search_content(query, limit=limit)

    # Format results for response
    formatted_results = []
    for result in results:
        formatted_results.append({
            "id": result.get("id"),
            "content": result.get("payload", {}).get("content", ""),
            "score": result.get("score"),
            "chapter_id": result.get("payload", {}).get("chapter_id"),
            "section": result.get("payload", {}).get("section", "")
        })

    return formatted_results


@router.post("/chat-sessions/{session_id}/generate-response")
async def generate_standalone_response(
    session_id: int,
    query: str,
    context_chapters: Optional[List[int]] = None,
    token: str = Depends(oauth2_scheme),
    db: Session = Depends(get_db)
):
    """
    Generate an AI response to a query without adding it to the session
    """
    # Verify token
    if not token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not authenticated",
            headers={"WWW-Authenticate": "Bearer"},
        )

    ai_service = get_ai_service(db)
    response = ai_service.generate_ai_response(
        query=query,
        session_id=session_id,
        context_chapters=context_chapters
    )

    return {"response": response}


# Include this router in the main app
# This would be done in main.py: app.include_router(ai_routes.router, prefix="/api/v1/ai", tags=["ai"])
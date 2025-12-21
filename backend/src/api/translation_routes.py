from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import Dict, Optional

from ..models.user import User
from ..models.chapter import Chapter
from ..services.translation_service import get_translation_service
from ..lib.database import get_db
from .user_routes import get_current_user_from_token

# Create router
router = APIRouter(
    prefix="/translation",
    tags=["translation"],
    responses={404: {"description": "Not found"}}
)


@router.get("/available/{chapter_id}")
async def get_available_translations(
    chapter_id: int,
    current_user: User = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Get available translations for a chapter
    """
    translation_service = get_translation_service(db)

    translations = translation_service.get_available_translations(chapter_id)

    if not translations:
        raise HTTPException(status_code=404, detail="Chapter not found")

    return {
        "chapter_id": chapter_id,
        "available_translations": list(translations.keys()),
        "translations": translations
    }


@router.post("/translate/{chapter_id}")
async def translate_chapter(
    chapter_id: int,
    target_language: str = "ur",
    current_user: User = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Translate a chapter to the target language
    """
    translation_service = get_translation_service(db)

    translated_chapter = translation_service.translate_chapter(chapter_id, target_language)

    if not translated_chapter:
        raise HTTPException(status_code=404, detail="Chapter not found or translation not available")

    return translated_chapter


@router.get("/supported-languages")
async def get_supported_languages(
    current_user: User = Depends(get_current_user_from_token)
):
    """
    Get list of supported languages for translation
    """
    return {
        "supported_languages": [
            {
                "code": "ur",
                "name": "Urdu",
                "native_name": "اردو",
                "status": "beta"
            },
            {
                "code": "en",
                "name": "English",
                "native_name": "English",
                "status": "native"
            }
        ]
    }


@router.post("/content")
async def translate_content(
    content: str,
    target_language: str = "ur",
    current_user: User = Depends(get_current_user_from_token),
    db: Session = Depends(get_db)
):
    """
    Translate arbitrary content to target language
    """
    if target_language != "ur":
        raise HTTPException(status_code=400, detail="Only Urdu translation is currently supported")

    translation_service = get_translation_service(db)
    translated_content = translation_service.add_translation_support(content, target_language)

    return {
        "original_content": content,
        "translated_content": translated_content,
        "source_language": "en",
        "target_language": target_language
    }
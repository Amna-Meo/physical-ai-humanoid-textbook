from typing import Optional, List
from sqlalchemy.orm import Session
from ..models.chapter import Chapter


class ChapterService:
    """
    Service for chapter-related operations
    """

    def __init__(self, db: Session):
        self.db = db

    def get_chapter(self, chapter_id: int) -> Optional[Chapter]:
        """
        Get a chapter by ID
        """
        return self.db.query(Chapter).filter(Chapter.id == chapter_id).first()

    def get_chapter_by_slug(self, slug: str) -> Optional[Chapter]:
        """
        Get a chapter by slug
        """
        return self.db.query(Chapter).filter(Chapter.slug == slug).first()

    def get_all_chapters(self, skip: int = 0, limit: int = 100) -> List[Chapter]:
        """
        Get all chapters with pagination
        """
        return self.db.query(Chapter).offset(skip).limit(limit).all()

    def create_chapter(self, chapter: Chapter) -> Chapter:
        """
        Create a new chapter
        """
        self.db.add(chapter)
        self.db.commit()
        self.db.refresh(chapter)
        return chapter

    def update_chapter(self, chapter_id: int, update_data: dict) -> Optional[Chapter]:
        """
        Update a chapter
        """
        chapter = self.get_chapter(chapter_id)
        if not chapter:
            return None

        for field, value in update_data.items():
            if hasattr(chapter, field):
                setattr(chapter, field, value)

        self.db.commit()
        self.db.refresh(chapter)
        return chapter

    def delete_chapter(self, chapter_id: int) -> bool:
        """
        Delete a chapter
        """
        chapter = self.get_chapter(chapter_id)
        if not chapter:
            return False

        self.db.delete(chapter)
        self.db.commit()
        return True

    def search_chapters(self, query: str) -> List[Chapter]:
        """
        Search chapters by title or content
        """
        return self.db.query(Chapter).filter(
            Chapter.title.contains(query) |
            Chapter.content.contains(query)
        ).all()


# Convenience function to get chapter service
def get_chapter_service(db: Session) -> ChapterService:
    return ChapterService(db)
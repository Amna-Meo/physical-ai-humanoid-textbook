from typing import Optional, List, Dict, Any
from sqlalchemy.orm import Session
from sqlalchemy import desc, asc
from datetime import datetime
from ..models.content_version import ContentVersion, ContentChangeLog
from ..models.chapter import Chapter
from ..models.user import User
import difflib
import json


class ContentVersioningService:
    """
    Service for content versioning and change tracking
    """

    def __init__(self, db: Session):
        self.db = db

    def create_content_version(
        self,
        content_id: int,
        content_type: str,
        title: Optional[str],
        content: Optional[str],
        author_id: Optional[int],
        change_summary: Optional[str] = None,
        change_type: str = "update"
    ) -> ContentVersion:
        """
        Create a new version of content
        """
        # Get the latest version number for this content
        latest_version = self.db.query(ContentVersion).filter(
            ContentVersion.content_id == content_id,
            ContentVersion.content_type == content_type
        ).order_by(desc(ContentVersion.version_number)).first()

        next_version_number = 1
        if latest_version:
            next_version_number = latest_version.version_number + 1

        # Create new version
        content_version = ContentVersion(
            content_id=content_id,
            content_type=content_type,
            version_number=next_version_number,
            title=title,
            content=content,
            summary=change_summary,
            author_id=author_id,
            change_type=change_type
        )

        self.db.add(content_version)
        self.db.commit()
        self.db.refresh(content_version)

        return content_version

    def get_content_history(
        self,
        content_id: int,
        content_type: str,
        limit: int = 50
    ) -> List[ContentVersion]:
        """
        Get the version history for content
        """
        return self.db.query(ContentVersion).filter(
            ContentVersion.content_id == content_id,
            ContentVersion.content_type == content_type
        ).order_by(desc(ContentVersion.version_number)).limit(limit).all()

    def get_content_at_version(
        self,
        content_id: int,
        content_type: str,
        version_number: int
    ) -> Optional[ContentVersion]:
        """
        Get content at a specific version
        """
        return self.db.query(ContentVersion).filter(
            ContentVersion.content_id == content_id,
            ContentVersion.content_type == content_type,
            ContentVersion.version_number == version_number
        ).first()

    def get_latest_version(self, content_id: int, content_type: str) -> Optional[ContentVersion]:
        """
        Get the latest version of content
        """
        return self.db.query(ContentVersion).filter(
            ContentVersion.content_id == content_id,
            ContentVersion.content_type == content_type
        ).order_by(desc(ContentVersion.version_number)).first()

    def compare_versions(
        self,
        content_id: int,
        content_type: str,
        version1: int,
        version2: int
    ) -> Dict[str, Any]:
        """
        Compare two versions of content and return differences
        """
        ver1 = self.get_content_at_version(content_id, content_type, version1)
        ver2 = self.get_content_at_version(content_id, content_type, version2)

        if not ver1 or not ver2:
            return {"error": "One or both versions not found"}

        # Compare content using difflib
        content1_lines = ver1.content.splitlines() if ver1.content else []
        content2_lines = ver2.content.splitlines() if ver2.content else []

        diff = list(difflib.unified_diff(
            content1_lines,
            content2_lines,
            fromfile=f'Version {version1}',
            tofile=f'Version {version2}',
            lineterm=''
        ))

        # Compare titles
        title_changed = ver1.title != ver2.title
        title_diff = {
            "old": ver1.title,
            "new": ver2.title,
            "changed": title_changed
        } if ver1.title or ver2.title else None

        return {
            "version1": version1,
            "version2": version2,
            "title_diff": title_diff,
            "content_diff": diff,
            "has_changes": len(diff) > 0 or title_changed
        }

    def revert_to_version(
        self,
        content_id: int,
        content_type: str,
        version_number: int,
        user_id: int,
        reason: str = "Revert to previous version"
    ) -> Optional[ContentVersion]:
        """
        Revert content to a previous version
        """
        target_version = self.get_content_at_version(content_id, content_type, version_number)
        if not target_version:
            return None

        # Create a new version with the reverted content
        reverted_version = self.create_content_version(
            content_id=content_id,
            content_type=content_type,
            title=target_version.title,
            content=target_version.content,
            author_id=user_id,
            change_summary=f"Reverted to version {version_number}: {reason}",
            change_type="revert"
        )

        return reverted_version

    def log_content_change(
        self,
        content_id: int,
        content_type: str,
        user_id: Optional[int],
        action: str,
        field_changed: Optional[str] = None,
        old_value: Optional[str] = None,
        new_value: Optional[str] = None,
        change_summary: Optional[str] = None,
        ip_address: Optional[str] = None,
        user_agent: Optional[str] = None
    ) -> ContentChangeLog:
        """
        Log a content change for audit trail
        """
        change_log = ContentChangeLog(
            content_id=content_id,
            content_type=content_type,
            user_id=user_id,
            action=action,
            field_changed=field_changed,
            old_value=old_value,
            new_value=new_value,
            change_summary=change_summary,
            ip_address=ip_address,
            user_agent=user_agent
        )

        self.db.add(change_log)
        self.db.commit()
        self.db.refresh(change_log)

        return change_log

    def get_change_log(
        self,
        content_id: Optional[int] = None,
        content_type: Optional[str] = None,
        user_id: Optional[int] = None,
        action: Optional[str] = None,
        start_date: Optional[datetime] = None,
        end_date: Optional[datetime] = None,
        limit: int = 100
    ) -> List[ContentChangeLog]:
        """
        Get change log entries with optional filters
        """
        query = self.db.query(ContentChangeLog)

        if content_id is not None:
            query = query.filter(ContentChangeLog.content_id == content_id)

        if content_type is not None:
            query = query.filter(ContentChangeLog.content_type == content_type)

        if user_id is not None:
            query = query.filter(ContentChangeLog.user_id == user_id)

        if action is not None:
            query = query.filter(ContentChangeLog.action == action)

        if start_date is not None:
            query = query.filter(ContentChangeLog.timestamp >= start_date)

        if end_date is not None:
            query = query.filter(ContentChangeLog.timestamp <= end_date)

        return query.order_by(desc(ContentChangeLog.timestamp)).limit(limit).all()

    def get_content_statistics(self, content_id: int, content_type: str) -> Dict[str, Any]:
        """
        Get statistics about content changes
        """
        total_versions = self.db.query(ContentVersion).filter(
            ContentVersion.content_id == content_id,
            ContentVersion.content_type == content_type
        ).count()

        change_logs = self.db.query(ContentChangeLog).filter(
            ContentChangeLog.content_id == content_id,
            ContentChangeLog.content_type == content_type
        ).all()

        # Count different types of actions
        action_counts = {}
        for log in change_logs:
            action_counts[log.action] = action_counts.get(log.action, 0) + 1

        # Get time range
        first_change = self.db.query(ContentChangeLog).filter(
            ContentChangeLog.content_id == content_id,
            ContentChangeLog.content_type == content_type
        ).order_by(asc(ContentChangeLog.timestamp)).first()

        last_change = self.db.query(ContentChangeLog).filter(
            ContentChangeLog.content_id == content_id,
            ContentChangeLog.content_type == content_type
        ).order_by(desc(ContentChangeLog.timestamp)).first()

        return {
            "total_versions": total_versions,
            "total_changes": len(change_logs),
            "action_counts": action_counts,
            "first_change": first_change.timestamp if first_change else None,
            "last_change": last_change.timestamp if last_change else None,
            "change_frequency": len(change_logs) / 7 if total_versions > 0 else 0  # Changes per week
        }

    def track_chapter_changes(self, chapter: Chapter, user_id: int, change_type: str = "update") -> ContentVersion:
        """
        Track changes to a chapter
        """
        # Create version record
        version = self.create_content_version(
            content_id=chapter.id,
            content_type="chapter",
            title=chapter.title,
            content=chapter.content,
            author_id=user_id,
            change_type=change_type
        )

        # Log the change
        self.log_content_change(
            content_id=chapter.id,
            content_type="chapter",
            user_id=user_id,
            action=change_type,
            field_changed="content",
            old_value=None,  # Would need to store previous version to compare
            new_value=chapter.content[:100] + "..." if chapter.content else None,  # First 100 chars
            change_summary=f"Chapter {change_type}: {chapter.title}"
        )

        return version

    def get_version_diff_for_frontend(
        self,
        content_id: int,
        content_type: str,
        from_version: int,
        to_version: int
    ) -> Dict[str, Any]:
        """
        Get version differences formatted for frontend display
        """
        comparison = self.compare_versions(content_id, content_type, from_version, to_version)

        if "error" in comparison:
            return comparison

        # Format the diff for frontend consumption
        formatted_diff = {
            "version1": from_version,
            "version2": to_version,
            "title_changed": comparison.get("title_diff", {}).get("changed", False),
            "title_diff": comparison.get("title_diff"),
            "content_changes": [],
            "has_changes": comparison.get("has_changes", False)
        }

        # Process the unified diff for frontend display
        diff_lines = comparison.get("content_diff", [])
        added_lines = []
        removed_lines = []
        context_lines = []

        for line in diff_lines:
            if line.startswith('+'):
                added_lines.append(line[1:])  # Remove the + sign
            elif line.startswith('-'):
                removed_lines.append(line[1:])  # Remove the - sign
            elif line.startswith('@@'):
                context_lines.append(line)

        formatted_diff["content_changes"] = {
            "added_lines": added_lines,
            "removed_lines": removed_lines,
            "context": context_lines,
            "total_changes": len(added_lines) + len(removed_lines)
        }

        return formatted_diff


# Convenience function to get content versioning service
def get_content_versioning_service(db: Session) -> ContentVersioningService:
    return ContentVersioningService(db)
from typing import List, Dict, Any, Optional
from sqlalchemy.orm import Session
import json
import zipfile
import os
from datetime import datetime
import markdown
from weasyprint import HTML, CSS
from io import BytesIO
import tempfile
from ..models.chapter import Chapter
from ..models.learning_path import LearningPath, LearningPathStep
from ..models.user import User
from ..models.ai_interaction import AIChatMessage


class ExportService:
    """
    Service for exporting content for offline learning
    """

    def __init__(self, db: Session):
        self.db = db

    def export_chapter_to_pdf(self, chapter_id: int, user_id: Optional[int] = None) -> BytesIO:
        """
        Export a single chapter to PDF format
        """
        chapter = self.db.query(Chapter).filter(Chapter.id == chapter_id).first()
        if not chapter:
            raise ValueError(f"Chapter with ID {chapter_id} not found")

        # Convert markdown content to HTML
        html_content = markdown.markdown(chapter.content, extensions=['tables', 'fenced_code'])

        # Create full HTML document
        full_html = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <meta charset="utf-8">
            <title>{chapter.title}</title>
            <style>
                body {{ font-family: Arial, sans-serif; margin: 40px; line-height: 1.6; }}
                h1, h2, h3, h4, h5, h6 {{ color: #333; }}
                code {{ background-color: #f4f4f4; padding: 2px 4px; border-radius: 3px; }}
                pre {{ background-color: #f4f4f4; padding: 10px; border-radius: 5px; overflow-x: auto; }}
                table {{ border-collapse: collapse; width: 100%; }}
                th, td {{ border: 1px solid #ddd; padding: 8px; text-align: left; }}
                th {{ background-color: #f2f2f2; }}
            </style>
        </head>
        <body>
            <h1>{chapter.title}</h1>
            <div class="metadata">
                <p><strong>Chapter:</strong> {chapter.chapter_number}</p>
                <p><strong>Word Count:</strong> {chapter.word_count}</p>
                <p><strong>Reading Time:</strong> ~{chapter.reading_time_minutes} minutes</p>
                <p><strong>Exported:</strong> {datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S')} UTC</p>
            </div>
            <div class="content">
                {html_content}
            </div>
        </body>
        </html>
        """

        # Convert to PDF
        html_doc = HTML(string=full_html)
        css = CSS(string='''
            @page {
                margin: 2cm;
                @bottom-right {
                    content: "Page " counter(page) " of " counter(pages);
                    font-size: 9pt;
                    font-style: italic;
                }
            }
            body {
                font-family: "Georgia", serif;
                line-height: 1.6;
                font-size: 12pt;
            }
            h1 { font-size: 18pt; }
            h2 { font-size: 16pt; }
            h3 { font-size: 14pt; }
        ''')

        pdf_bytes = html_doc.write_pdf(stylesheets=[css])
        return BytesIO(pdf_bytes)

    def export_chapter_to_markdown(self, chapter_id: int) -> str:
        """
        Export a chapter to markdown format
        """
        chapter = self.db.query(Chapter).filter(Chapter.id == chapter_id).first()
        if not chapter:
            raise ValueError(f"Chapter with ID {chapter_id} not found")

        markdown_content = f"""# {chapter.title}

**Chapter Number:** {chapter.chapter_number}
**Word Count:** {chapter.word_count}
**Reading Time:** ~{chapter.reading_time_minutes} minutes
**Exported:** {datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S')} UTC

---

{chapter.content}
"""
        return markdown_content

    def export_learning_path_to_zip(self, learning_path_id: int, user_id: Optional[int] = None) -> BytesIO:
        """
        Export an entire learning path as a ZIP file containing all chapters
        """
        learning_path = self.db.query(LearningPath).filter(LearningPath.id == learning_path_id).first()
        if not learning_path:
            raise ValueError(f"Learning path with ID {learning_path_id} not found")

        # Get all steps in the learning path
        steps = self.db.query(LearningPathStep).filter(
            LearningPathStep.learning_path_id == learning_path_id
        ).order_by(LearningPathStep.step_number).all()

        # Create a ZIP file in memory
        zip_buffer = BytesIO()
        with zipfile.ZipFile(zip_buffer, 'w', zipfile.ZIP_DEFLATED) as zip_file:
            # Add learning path metadata
            metadata = {
                "id": learning_path.id,
                "title": learning_path.title,
                "description": learning_path.description,
                "difficulty_level": learning_path.difficulty_level,
                "estimated_duration_hours": learning_path.estimated_duration_hours,
                "created_at": learning_path.created_at.isoformat() if learning_path.created_at else None,
                "exported_at": datetime.utcnow().isoformat(),
                "total_steps": len(steps),
                "steps": []
            }

            for step in steps:
                if step.content_type == "chapter" and step.content_id:
                    # Get the chapter
                    chapter = self.db.query(Chapter).filter(Chapter.id == step.content_id).first()
                    if chapter:
                        # Add chapter as markdown
                        chapter_md = self.export_chapter_to_markdown(chapter.id)
                        zip_file.writestr(f"chapter_{step.step_number:02d}_{chapter.slug}.md", chapter_md)

                        # Also add as PDF
                        pdf_bytes = self.export_chapter_to_pdf(chapter.id)
                        zip_file.writestr(f"chapter_{step.step_number:02d}_{chapter.slug}.pdf", pdf_bytes.read())

                        # Add to metadata
                        metadata["steps"].append({
                            "step_number": step.step_number,
                            "title": chapter.title,
                            "chapter_id": chapter.id,
                            "estimated_duration_minutes": step.estimated_duration_minutes,
                            "required": step.required
                        })

            # Write metadata
            zip_file.writestr("learning_path_metadata.json", json.dumps(metadata, indent=2))

            # Write table of contents
            toc_content = f"# {learning_path.title}\n\n"
            toc_content += f"## Table of Contents\n\n"
            for step in metadata["steps"]:
                toc_content += f"{step['step_number']}. [{step['title']}](./chapter_{step['step_number']:02d}_{step['title'].lower().replace(' ', '_')}.md)\n"

            zip_file.writestr("toc.md", toc_content)

        # Seek to beginning of buffer
        zip_buffer.seek(0)
        return zip_buffer

    def export_user_learning_data(self, user_id: int) -> BytesIO:
        """
        Export all user learning data including progress, notes, and interactions
        """
        user = self.db.query(User).filter(User.id == user_id).first()
        if not user:
            raise ValueError(f"User with ID {user_id} not found")

        # Get user's learning paths
        from .learning_path_service import get_learning_path_service
        lp_service = get_learning_path_service(self.db)
        user_learning_paths = lp_service.get_user_learning_paths(user_id)

        # Get user's AI interactions
        ai_messages = self.db.query(AIChatMessage).filter(
            AIChatMessage.session.has(user_id=user_id)  # Assuming there's a session relationship
        ).order_by(AIChatMessage.timestamp).all()

        # Create export data
        export_data = {
            "user": {
                "id": user.id,
                "username": user.username,
                "email": user.email,
                "full_name": user.full_name,
                "created_at": user.created_at.isoformat() if user.created_at else None,
                "exported_at": datetime.utcnow().isoformat()
            },
            "learning_paths": [],
            "ai_interactions": [],
            "progress_data": {}
        }

        # Add learning path data
        for user_lp in user_learning_paths:
            lp_data = {
                "id": user_lp.id,
                "learning_path_id": user_lp.learning_path_id,
                "title": user_lp.learning_path.title if user_lp.learning_path else "Unknown",
                "status": user_lp.status,
                "current_step": user_lp.current_step,
                "progress_percentage": user_lp.progress_percentage,
                "created_at": user_lp.created_at.isoformat() if user_lp.created_at else None,
                "completed_at": user_lp.completed_at.isoformat() if user_lp.completed_at else None
            }
            export_data["learning_paths"].append(lp_data)

        # Add AI interaction data
        for msg in ai_messages:
            ai_data = {
                "id": msg.id,
                "session_id": msg.session_id,
                "role": msg.role,
                "content": msg.content,
                "timestamp": msg.timestamp.isoformat() if msg.timestamp else None
            }
            export_data["ai_interactions"].append(ai_data)

        # Create ZIP file with export data
        zip_buffer = BytesIO()
        with zipfile.ZipFile(zip_buffer, 'w', zipfile.ZIP_DEFLATED) as zip_file:
            # Add main export data
            zip_file.writestr("user_learning_data.json", json.dumps(export_data, indent=2))

            # Add individual components
            zip_file.writestr("learning_paths.json", json.dumps(export_data["learning_paths"], indent=2))
            zip_file.writestr("ai_interactions.json", json.dumps(export_data["ai_interactions"], indent=2))

            # Create a readable summary
            summary = f"""User Learning Data Export
========================

User: {user.full_name or user.username} ({user.email})
Export Date: {datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S')} UTC

Learning Paths: {len(export_data['learning_paths'])}
AI Interactions: {len(export_data['ai_interactions'])}

Detailed data is available in the JSON files included in this export.
"""
            zip_file.writestr("summary.txt", summary)

        zip_buffer.seek(0)
        return zip_buffer

    def export_chapter_to_epub(self, chapter_id: int) -> BytesIO:
        """
        Export a chapter to EPUB format for e-readers
        """
        chapter = self.db.query(Chapter).filter(Chapter.id == chapter_id).first()
        if not chapter:
            raise ValueError(f"Chapter with ID {chapter_id} not found")

        # Create EPUB content
        epub_content = f"""<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE html>
<html xmlns="http://www.w3.org/1999/xhtml" xmlns:epub="http://www.idpf.org/2007/ops">
<head>
    <title>{chapter.title}</title>
    <meta charset="UTF-8"/>
</head>
<body>
    <header>
        <h1>{chapter.title}</h1>
        <p>Chapter {chapter.chapter_number}</p>
        <p>Exported: {datetime.utcnow().strftime('%Y-%m-%d')}</p>
    </header>
    <main>
        {markdown.markdown(chapter.content, extensions=['tables', 'fenced_code'])}
    </main>
</body>
</html>"""

        # In a real implementation, we would create a proper EPUB file
        # For now, we'll return the HTML content in a ZIP file with EPUB extension
        zip_buffer = BytesIO()
        with zipfile.ZipFile(zip_buffer, 'w', zipfile.ZIP_DEFLATED) as zip_file:
            zip_file.writestr("chapter.html", epub_content)
            # Add basic EPUB structure files
            zip_file.writestr("mimetype", "application/epub+zip")
            zip_file.writestr("META-INF/container.xml", """<?xml version="1.0"?>
<container version="1.0" xmlns="urn:oasis:names:tc:opendocument:xmlns:container">
   <rootfiles>
      <rootfile full-path="OEBPS/content.opf" media-type="application/oebps-package+xml"/>
   </rootfiles>
</container>""")

        zip_buffer.seek(0)
        return zip_buffer

    def get_export_formats(self) -> List[Dict[str, str]]:
        """
        Get available export formats
        """
        return [
            {
                "format": "pdf",
                "name": "PDF",
                "description": "Portable Document Format, good for printing and sharing",
                "file_extension": ".pdf",
                "use_case": "Offline reading, printing, sharing with others"
            },
            {
                "format": "markdown",
                "name": "Markdown",
                "description": "Plain text format with simple formatting",
                "file_extension": ".md",
                "use_case": "Editing, version control, compatibility with various tools"
            },
            {
                "format": "zip",
                "name": "ZIP Archive",
                "description": "Compressed archive containing multiple files",
                "file_extension": ".zip",
                "use_case": "Complete learning path with all materials"
            },
            {
                "format": "epub",
                "name": "EPUB",
                "description": "Electronic publication format for e-readers",
                "file_extension": ".epub",
                "use_case": "Reading on e-readers and mobile devices"
            }
        ]


# Convenience function to get export service
def get_export_service(db: Session) -> ExportService:
    return ExportService(db)
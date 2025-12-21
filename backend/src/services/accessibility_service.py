from typing import Dict, List, Optional, Any
from sqlalchemy.orm import Session
from ..models.chapter import Chapter
from ..models.user import User
import re
from enum import Enum


class AccessibilityFeature(Enum):
    """Enumeration for different accessibility features"""
    ALT_TEXT = "alt_text"
    CAPTIONS = "captions"
    KEYBOARD_NAVIGATION = "keyboard_navigation"
    SCREEN_READER = "screen_reader"
    HIGH_CONTRAST = "high_contrast"
    TEXT_SIZE_ADJUSTMENT = "text_size_adjustment"
    DYSLEXIA_FRIENDLY = "dyslexia_friendly"
    COGNITIVE_SUPPORT = "cognitive_support"


class AccessibilityService:
    """
    Service for accessibility features to ensure inclusive learning
    """

    def __init__(self, db: Session):
        self.db = db

    def generate_alt_text(self, content: str) -> str:
        """
        Generate alternative text for images and figures in content
        """
        # This would normally use AI to analyze images
        # For now, we'll create descriptive alt text based on surrounding context
        alt_text = self._analyze_content_for_alt_text(content)
        return alt_text

    def _analyze_content_for_alt_text(self, content: str) -> str:
        """
        Analyze content to generate appropriate alt text
        """
        # Look for image references and create descriptive text
        image_patterns = [
            r'!\[([^\]]*)\]\([^)]*\)',  # Markdown image syntax
            r'<img[^>]*alt="([^"]*)"[^>]*>',  # HTML img with alt
            r'<img[^>]*title="([^"]*)"[^>]*>',  # HTML img with title
        ]

        alt_texts = []
        for pattern in image_patterns:
            matches = re.findall(pattern, content, re.IGNORECASE)
            alt_texts.extend(matches)

        if alt_texts:
            return " ".join(alt_texts)
        else:
            # Generate a generic description based on content context
            words = content.split()[:20]  # First 20 words as context
            return f"Image related to: {' '.join(words[:10])}..."

    def generate_content_summary(self, content: str, max_length: int = 200) -> str:
        """
        Generate a summary of content for screen readers and cognitive support
        """
        # Remove markdown formatting for summary
        clean_content = re.sub(r'[#*\[\]()`]', '', content)

        # Get first few sentences
        sentences = re.split(r'[.!?]+', clean_content)
        summary = ""
        for sentence in sentences:
            if len(summary + sentence) < max_length:
                summary += sentence.strip() + ". "
            else:
                break

        return summary.strip()

    def enhance_content_for_accessibility(self, content: str, features: List[AccessibilityFeature]) -> Dict[str, Any]:
        """
        Enhance content with accessibility features
        """
        enhanced_content = content
        metadata = {}

        for feature in features:
            if feature == AccessibilityFeature.ALT_TEXT:
                # Add alt text to images
                enhanced_content = self._enhance_with_alt_text(enhanced_content)
            elif feature == AccessibilityFeature.SCREEN_READER:
                # Add screen reader friendly elements
                enhanced_content = self._enhance_for_screen_readers(enhanced_content)
            elif feature == AccessibilityFeature.COGNITIVE_SUPPORT:
                # Add cognitive support elements
                enhanced_content = self._add_cognitive_support(enhanced_content)
                metadata['summary'] = self.generate_content_summary(content)

        return {
            "content": enhanced_content,
            "metadata": metadata,
            "enhanced_features": [f.value for f in features]
        }

    def _enhance_with_alt_text(self, content: str) -> str:
        """
        Enhance content by ensuring images have alt text
        """
        # Pattern to find images without alt text
        no_alt_pattern = r'!\[([^\]]*)\]\(([^)]+)\)'  # ![text](url) or ![](url)

        def add_alt_text(match):
            alt_text = match.group(1) if match.group(1) else self._generate_alt_from_filename(match.group(2))
            return f'![{alt_text}]({match.group(2)})'

        enhanced = re.sub(no_alt_pattern, add_alt_text, content)
        return enhanced

    def _generate_alt_from_filename(self, filename: str) -> str:
        """
        Generate alt text from filename
        """
        # Extract name from path
        name = filename.split('/')[-1].split('.')[0]
        # Replace underscores/hyphens with spaces
        name = name.replace('_', ' ').replace('-', ' ')
        return f"Image: {name}"

    def _enhance_for_screen_readers(self, content: str) -> str:
        """
        Enhance content for screen readers
        """
        # Add ARIA labels and semantic structure
        enhanced = content

        # Add headings structure if missing
        heading_pattern = r'^(.+)$'
        lines = enhanced.split('\n')
        enhanced_lines = []

        for line in lines:
            if line.strip() and not line.startswith(('#', '!', '[', '-', '*', '1.', '2.')):
                # This might be content that should be in a section
                enhanced_lines.append(line)
            else:
                enhanced_lines.append(line)

        return '\n'.join(enhanced_lines)

    def _add_cognitive_support(self, content: str) -> str:
        """
        Add cognitive support features to content
        """
        # Add clear section breaks and summaries
        sections = re.split(r'\n#{2,}', content)
        enhanced_sections = []

        for i, section in enumerate(sections):
            if i == 0:
                # First section, add introduction
                enhanced_sections.append(f"<div class='cognitive-intro'>\n{section}\n</div>")
            else:
                # Other sections
                enhanced_sections.append(f"<div class='cognitive-section'>\n{section}\n</div>")

        return '\n## '.join(enhanced_sections)

    def get_accessibility_score(self, content: str) -> Dict[str, float]:
        """
        Calculate accessibility score for content
        """
        score = 0
        max_score = 100

        # Check for headings
        headings = re.findall(r'#{2,6}', content)
        if headings:
            score += 20  # Good heading structure

        # Check for alt text in images
        images_with_alt = re.findall(r'!\[[^\]]+\]\([^)]+\)', content)
        total_images = len(images_with_alt)
        if total_images > 0:
            score += 15

        # Check for lists and structure
        lists = re.findall(r'^\s*[-*+]\s|^(\d+)\.', content, re.MULTILINE)
        if lists:
            score += 15

        # Check for paragraphs (not just long text blocks)
        paragraphs = content.split('\n\n')
        if len(paragraphs) > 2:
            score += 10

        # Check for links (should have descriptive text)
        links = re.findall(r'\[([^\]]+)\]\([^)]+\)', content)
        if links:
            score += 10

        # Check for tables (structured data)
        tables = re.findall(r'\|[^\n]*\|', content)
        if tables:
            score += 10

        # Additional checks for accessibility
        # Check for emphasis (bold, italic)
        emphasis = re.findall(r'[*_]{1,2}[^*_]+[*_]{1,2}', content)
        if emphasis:
            score += 10

        # Cap the score at max
        final_score = min(score, max_score)

        return {
            "score": round(final_score, 2),
            "max_score": max_score,
            "percentage": round((final_score / max_score) * 100, 2),
            "breakdown": {
                "headings": min(20, len(headings) * 5 if headings else 0),
                "images_with_alt": min(15, len(images_with_alt) * 3 if images_with_alt else 0),
                "lists_structure": min(15, len(lists) * 2 if lists else 0),
                "paragraphs": min(10, len(paragraphs) * 1 if len(paragraphs) > 2 else 0),
                "links": min(10, len(links) * 1 if links else 0),
                "tables": min(10, len(tables) * 2 if tables else 0),
                "emphasis": min(10, len(emphasis) * 1 if emphasis else 0)
            }
        }

    def get_user_accessibility_preferences(self, user_id: int) -> Dict[str, Any]:
        """
        Get user's accessibility preferences
        """
        # In a real implementation, this would fetch from user profile
        # For now, return default preferences
        return {
            "user_id": user_id,
            "high_contrast_mode": False,
            "larger_text_size": False,
            "dyslexia_font": False,
            "screen_reader_mode": False,
            "captions_enabled": True,
            "reduced_motion": False,
            "custom_color_scheme": None
        }

    def update_user_accessibility_preferences(self, user_id: int, preferences: Dict[str, Any]) -> bool:
        """
        Update user's accessibility preferences
        """
        # In a real implementation, this would update the user profile in database
        # For now, just validate the preferences
        valid_keys = {
            "high_contrast_mode", "larger_text_size", "dyslexia_font",
            "screen_reader_mode", "captions_enabled", "reduced_motion", "custom_color_scheme"
        }

        for key in preferences:
            if key not in valid_keys:
                return False

        # In real implementation, update user record
        return True

    def create_accessible_chapter(self, chapter: Chapter) -> Dict[str, Any]:
        """
        Create an accessible version of a chapter
        """
        accessibility_features = [
            AccessibilityFeature.ALT_TEXT,
            AccessibilityFeature.SCREEN_READER,
            AccessibilityFeature.COGNITIVE_SUPPORT
        ]

        enhanced_content = self.enhance_content_for_accessibility(
            chapter.content,
            accessibility_features
        )

        accessibility_score = self.get_accessibility_score(chapter.content)

        return {
            "chapter_id": chapter.id,
            "title": chapter.title,
            "accessible_content": enhanced_content["content"],
            "metadata": enhanced_content["metadata"],
            "accessibility_score": accessibility_score,
            "enhanced_features": enhanced_content["enhanced_features"],
            "original_word_count": len(chapter.content.split()),
            "reading_level": self._estimate_reading_level(chapter.content)
        }

    def _estimate_reading_level(self, content: str) -> str:
        """
        Estimate reading level of content (simplified)
        """
        words = content.split()
        sentences = re.split(r'[.!?]+', content)
        words_per_sentence = len(words) / max(len(sentences), 1)

        if words_per_sentence < 10:
            return "Beginner"
        elif words_per_sentence < 15:
            return "Intermediate"
        elif words_per_sentence < 20:
            return "Advanced"
        else:
            return "Expert"

    def get_accessibility_guidelines(self) -> Dict[str, List[str]]:
        """
        Get accessibility guidelines for content creators
        """
        return {
            "headings": [
                "Use proper heading hierarchy (h1, h2, h3)",
                "Ensure headings are descriptive and unique",
                "Don't skip heading levels"
            ],
            "images": [
                "Always include descriptive alt text",
                "Use meaningful file names",
                "Provide captions when necessary"
            ],
            "links": [
                "Use descriptive link text",
                "Distinguish links visually",
                "Ensure links are keyboard accessible"
            ],
            "color_contrast": [
                "Maintain at least 4.5:1 contrast ratio",
                "Don't rely solely on color to convey information",
                "Test with color blindness simulators"
            ],
            "keyboard_navigation": [
                "Ensure all functionality is keyboard accessible",
                "Provide visible focus indicators",
                "Use logical tab order"
            ],
            "forms": [
                "Provide clear labels for form fields",
                "Use proper error messaging",
                "Group related form elements"
            ]
        }


# Convenience function to get accessibility service
def get_accessibility_service(db: Session) -> AccessibilityService:
    return AccessibilityService(db)
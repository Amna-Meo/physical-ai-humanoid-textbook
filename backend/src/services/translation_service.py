from typing import Dict, List, Optional
import json
from ..models.chapter import Chapter
from sqlalchemy.orm import Session


class TranslationService:
    """
    Service for handling content translation, including Urdu translation functionality
    """

    def __init__(self, db: Session):
        self.db = db
        # Urdu translation dictionary - simplified for demonstration
        self.urdu_translations = {
            # Common terms in Physical AI context
            "Physical AI": "مادی مصنوعی ذہانت",
            "Humanoid Robotics": "ہیومنوائڈ روبوٹس",
            "Embodiment": "جثہ بدنی",
            "Real-time Processing": "ریل ٹائم پروسیسنگ",
            "Uncertainty Management": "اٹھل پٹھل کا نظم",
            "Energy Efficiency": "توانائی کی کارکردگی",
            "Locomotion": "چلنے پھرنے کا عمل",
            "Control Systems": "کنٹرول سسٹم",
            "Perception Systems": "ادراک سسٹم",
            "Planning and Navigation": " منصوبہ بندی اور نیویگیشن",
            "Integration and Deployment": "انضمام اور تعیناتی",
            "Robot Operating System": "روبوٹ آپریٹنگ سسٹم",
            "Simulation": "شبیہ سازی",
            "Vision-Language-Action": "دید-زبان-عمل",
            "Balance Control": " توازن کنٹرول",
            "ZMP": "ZMP",  # Keep acronym as is
            "Center of Mass": "ماس کا مرکز",
            "Stability": "مستحکم",
            "Dynamics": "مکانکات",
            "Kinematics": "کنیمیٹکس",

            # Common phrases
            "Introduction to": "کا تعارف",
            "Chapter": "باب",
            "Learning Objectives": "سیکھنے کے مقاصد",
            "Exercises": "مشقیں",
            "Further Reading": "مزید مطالعہ",
            "Applications": "اطلاق",
            "Challenges": "چیلنج",
            "Principles": "اصول",
            "Systems": "سسٹمز",
            "Control": "کنٹرول",
            "Navigation": "نیویگیشن",
            "Integration": "انضمام",
            "Deployment": "تعیناتی",
        }

    def translate_to_urdu(self, text: str) -> str:
        """
        Translate English text to Urdu using the translation dictionary
        This is a simplified implementation - in production, would use proper NLP models
        """
        # For now, we'll do a simple word replacement approach
        # In a real implementation, this would use proper NLP models
        translated_text = text

        # Sort keys by length (descending) to avoid partial replacements
        sorted_keys = sorted(self.urdu_translations.keys(), key=len, reverse=True)

        for key in sorted_keys:
            translated_text = translated_text.replace(key, self.urdu_translations[key])

        return translated_text

    def get_available_translations(self, chapter_id: int) -> Dict[str, str]:
        """
        Get available translations for a chapter
        """
        chapter = self.db.query(Chapter).filter(Chapter.id == chapter_id).first()
        if not chapter:
            return {}

        # Create Urdu translation
        urdu_content = self.translate_to_urdu(chapter.content)
        urdu_title = self.translate_to_urdu(chapter.title)

        return {
            "en": {
                "title": chapter.title,
                "content": chapter.content
            },
            "ur": {
                "title": urdu_title,
                "content": urdu_content
            }
        }

    def translate_chapter(self, chapter_id: int, target_language: str = "ur") -> Optional[Dict]:
        """
        Translate a chapter to the target language
        """
        if target_language != "ur":
            return None  # Currently only support Urdu

        chapter = self.db.query(Chapter).filter(Chapter.id == chapter_id).first()
        if not chapter:
            return None

        urdu_content = self.translate_to_urdu(chapter.content)
        urdu_title = self.translate_to_urdu(chapter.title)
        urdu_description = self.translate_to_urdu(chapter.description or "")

        return {
            "id": chapter.id,
            "original_language": "en",
            "target_language": target_language,
            "title": urdu_title,
            "description": urdu_description,
            "content": urdu_content,
            "word_count": len(urdu_content.split())
        }

    def add_translation_support(self, content: str, language: str = "ur") -> str:
        """
        Add translation support to content
        """
        if language == "ur":
            return self.translate_to_urdu(content)
        return content


# Convenience function to get translation service
def get_translation_service(db: Session) -> TranslationService:
    return TranslationService(db)
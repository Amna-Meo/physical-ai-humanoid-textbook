import json
from typing import List, Optional
from sqlalchemy.orm import Session

from ..models.user import User
from ..models.chapter import Chapter
from ..models.learning_path import LearningPath


class ContentRecommendationService:
    def __init__(self, db: Session):
        self.db = db

    def get_content_recommendations(self, user_id: int, limit: int = 10) -> List[dict]:
        """
        Get content recommendations based on user profile
        """
        user = self.db.query(User).filter(User.id == user_id).first()
        if not user:
            raise ValueError("User not found")

        # Get user profile information
        professional_background = user.professional_background or ""
        learning_preferences_str = user.learning_preferences or "{}"

        try:
            learning_preferences = json.loads(learning_preferences_str) if learning_preferences_str else {}
        except json.JSONDecodeError:
            learning_preferences = {}

        # Get all available chapters
        all_chapters = self.db.query(Chapter).filter(
            Chapter.is_published == True
        ).all()

        # Score chapters based on user profile
        scored_chapters = []
        for chapter in all_chapters:
            score = self._calculate_content_score(chapter, professional_background, learning_preferences)
            if score > 0:  # Only include chapters with positive scores
                scored_chapters.append({
                    'chapter': chapter,
                    'score': score
                })

        # Sort by score in descending order
        scored_chapters.sort(key=lambda x: x['score'], reverse=True)

        # Return top recommendations
        recommendations = []
        for item in scored_chapters[:limit]:
            chapter = item['chapter']
            recommendations.append({
                'id': chapter.id,
                'title': chapter.title,
                'slug': chapter.slug,
                'description': chapter.description,
                'reading_time_minutes': chapter.reading_time_minutes,
                'relevance_score': item['score'],
                'tags': self._get_chapter_tags(chapter)
            })

        return recommendations

    def _calculate_content_score(self, chapter: Chapter, professional_background: str, learning_preferences: dict) -> float:
        """
        Calculate relevance score for a chapter based on user profile
        """
        score = 0.0

        # Boost score based on keywords in professional background
        chapter_lower = f"{chapter.title} {chapter.description} {chapter.content}".lower()
        background_lower = professional_background.lower()

        # Keywords that might indicate relevance
        robotics_keywords = ['robot', 'robotic', 'locomotion', 'manipulation', 'control', 'dynamics', 'kinematics', 'humanoid']
        ai_keywords = ['ai', 'artificial intelligence', 'machine learning', 'neural', 'deep learning', 'reinforcement', 'planning']
        programming_keywords = ['python', 'c++', 'ros', 'simulation', 'algorithm', 'programming']
        math_keywords = ['math', 'calculus', 'linear algebra', 'optimization', 'probability', 'statistics']

        # Score based on content type preferences
        if learning_preferences.get('content_format'):
            content_formats = learning_preferences['content_format']
            if 'theoretical' in content_formats and any(keyword in chapter_lower for keyword in math_keywords):
                score += 2.0
            if 'practical' in content_formats and any(keyword in chapter_lower for keyword in programming_keywords):
                score += 2.0
            if 'hands_on' in content_formats and any(keyword in chapter_lower for keyword in ['simulation', 'exercise', 'project']):
                score += 2.0

        # Score based on professional background keywords
        if any(keyword in background_lower for keyword in robotics_keywords):
            if any(keyword in chapter_lower for keyword in robotics_keywords):
                score += 3.0

        if any(keyword in background_lower for keyword in ai_keywords):
            if any(keyword in chapter_lower for keyword in ai_keywords):
                score += 3.0

        if any(keyword in background_lower for keyword in programming_keywords):
            if any(keyword in chapter_lower for keyword in programming_keywords):
                score += 2.5

        # Score based on experience level
        years_experience = learning_preferences.get('years_experience', '0-1')
        difficulty_indicators = {
            'intro': ['introduction', 'basic', 'fundamentals', 'beginner'],
            'intermediate': ['intermediate', 'advanced concepts', 'practical'],
            'advanced': ['advanced', 'complex', 'research', 'cutting-edge']
        }

        chapter_lower = f"{chapter.title} {chapter.description}".lower()

        if years_experience in ['0-1', '1-3']:
            # Beginner professional gets boost for introductory content
            if any(keyword in chapter_lower for keyword in difficulty_indicators['intro']):
                score += 3.0
        elif years_experience in ['3-5', '5-10']:
            # Intermediate professional gets boost for intermediate content
            if any(keyword in chapter_lower for keyword in difficulty_indicators['intermediate']):
                score += 3.0
        else:
            # Experienced professional gets boost for advanced content
            if any(keyword in chapter_lower for keyword in difficulty_indicators['advanced']):
                score += 3.0

        # Boost for specific interests
        if learning_preferences.get('primary_goal') == 'career_advancement':
            if any(keyword in chapter_lower for keyword in ['application', 'industry', 'real-world', 'practice']):
                score += 1.5

        if learning_preferences.get('primary_goal') == 'research':
            if any(keyword in chapter_lower for keyword in ['research', 'study', 'analysis', 'theoretical']):
                score += 1.5

        # Normalize score based on chapter length and quality metrics
        if chapter.word_count and chapter.word_count > 1000:  # Longer, more comprehensive chapters
            score += 1.0

        if chapter.is_ai_optimized:  # AI-optimized content
            score += 0.5

        return score

    def _get_chapter_tags(self, chapter: Chapter) -> List[str]:
        """
        Extract tags from chapter content
        """
        tags = []
        content_lower = f"{chapter.title} {chapter.description} {chapter.content}".lower()

        # Define tag categories
        tag_mapping = {
            'Robotics': ['robot', 'robotic', 'locomotion', 'manipulation', 'control', 'dynamics', 'kinematics', 'humanoid'],
            'AI/ML': ['ai', 'artificial intelligence', 'machine learning', 'neural', 'deep learning', 'reinforcement', 'planning'],
            'Programming': ['python', 'c++', 'ros', 'simulation', 'algorithm', 'programming', 'code'],
            'Mathematics': ['math', 'calculus', 'linear algebra', 'optimization', 'probability', 'statistics', 'equation'],
            'Control Systems': ['control', 'feedback', 'pid', 'stability', 'dynamics'],
            'Computer Vision': ['vision', 'image', 'camera', 'object detection', 'perception'],
            'Navigation': ['navigation', 'path planning', 'mapping', 'localization', 'slam'],
            'Human-Robot Interaction': ['interaction', 'hri', 'social', 'collaboration', 'interface']
        }

        for tag, keywords in tag_mapping.items():
            if any(keyword in content_lower for keyword in keywords):
                tags.append(tag)

        return tags[:5]  # Limit to top 5 tags

    def get_learning_path_recommendations(self, user_id: int, limit: int = 5) -> List[dict]:
        """
        Get learning path recommendations based on user profile
        """
        user = self.db.query(User).filter(User.id == user_id).first()
        if not user:
            raise ValueError("User not found")

        # Get all available learning paths
        all_paths = self.db.query(LearningPath).filter(
            LearningPath.is_active == True,
            LearningPath.is_personalized == False  # Exclude personalized paths
        ).all()

        # Score learning paths based on user profile
        scored_paths = []
        for path in all_paths:
            score = self._calculate_learning_path_score(path, user)
            if score > 0:
                scored_paths.append({
                    'path': path,
                    'score': score
                })

        # Sort by score in descending order
        scored_paths.sort(key=lambda x: x['score'], reverse=True)

        # Return top recommendations
        recommendations = []
        for item in scored_paths[:limit]:
            path = item['path']
            recommendations.append({
                'id': path.id,
                'title': path.title,
                'description': path.description,
                'difficulty_level': path.difficulty_level,
                'estimated_duration_hours': path.estimated_duration_hours,
                'relevance_score': item['score']
            })

        return recommendations

    def _calculate_learning_path_score(self, path: LearningPath, user: User) -> float:
        """
        Calculate relevance score for a learning path based on user profile
        """
        score = 0.0

        # Get user information
        professional_background = user.professional_background or ""
        learning_preferences_str = user.learning_preferences or "{}"

        try:
            learning_preferences = json.loads(learning_preferences_str) if learning_preferences_str else {}
        except json.JSONDecodeError:
            learning_preferences = {}

        # Score based on difficulty match
        user_exp_level = self._get_user_experience_level(learning_preferences.get('years_experience', '0-1'))
        path_difficulty = path.difficulty_level or 'beginner'

        if user_exp_level == path_difficulty:
            score += 3.0
        elif self._is_appropriate_difficulty(user_exp_level, path_difficulty):
            score += 1.5

        # Score based on content type preferences
        if learning_preferences.get('primary_goal') == 'career_advancement':
            if 'career' in path.title.lower() or 'professional' in path.title.lower():
                score += 2.0

        if learning_preferences.get('primary_goal') == 'research':
            if 'research' in path.title.lower() or 'advanced' in path.title.lower():
                score += 2.0

        # Score based on time availability
        time_availability = learning_preferences.get('time_availability', 'less_2')
        if time_availability == 'less_2' and path.estimated_duration_hours and path.estimated_duration_hours <= 5:
            score += 2.0
        elif time_availability == '2_5' and path.estimated_duration_hours and 5 < path.estimated_duration_hours <= 15:
            score += 2.0
        elif time_availability == '5_10' and path.estimated_duration_hours and 15 < path.estimated_duration_hours <= 30:
            score += 2.0
        elif time_availability == '20_plus':
            score += 1.0  # Flexible for longer paths

        # Score based on professional background match
        background_lower = professional_background.lower()
        path_lower = f"{path.title} {path.description}".lower()

        if any(keyword in background_lower for keyword in ['robotics', 'engineer', 'developer']):
            if any(keyword in path_lower for keyword in ['robot', 'robotic', 'control', 'dynamics']):
                score += 2.5

        if any(keyword in background_lower for keyword in ['ai', 'ml', 'machine learning']):
            if any(keyword in path_lower for keyword in ['ai', 'ml', 'machine learning', 'neural']):
                score += 2.5

        return score

    def _get_user_experience_level(self, years_experience: str) -> str:
        """
        Convert years of experience to difficulty level
        """
        if years_experience in ['0-1', '1-3']:
            return 'beginner'
        elif years_experience in ['3-5', '5-10']:
            return 'intermediate'
        else:
            return 'advanced'

    def _is_appropriate_difficulty(self, user_level: str, path_level: str) -> bool:
        """
        Check if path difficulty is appropriate for user level (allowing one level up)
        """
        level_hierarchy = {'beginner': 0, 'intermediate': 1, 'advanced': 2}

        user_idx = level_hierarchy.get(user_level, 0)
        path_idx = level_hierarchy.get(path_level, 0)

        # User can handle their level or one level above
        return path_idx <= user_idx + 1


# Convenience function to get content recommendation service
def get_content_recommendation_service(db: Session) -> ContentRecommendationService:
    return ContentRecommendationService(db)
from typing import List, Optional, Dict, Any
from sqlalchemy.orm import Session
from datetime import datetime, timedelta
import json

from ..models.user import User
from ..models.chapter import Chapter
from ..models.learning_path import UserLearningPath, UserLearningPathProgress, LearningPathStep
from ..models.ai_interaction import AIChatSession, AIChatMessage


class AdaptiveContentService:
    def __init__(self, db: Session):
        self.db = db

    def get_adaptive_content_for_user(self, user_id: int, current_chapter_id: Optional[int] = None) -> Dict[str, Any]:
        """
        Get adaptive content recommendations based on user's progress and performance
        """
        user = self.db.query(User).filter(User.id == user_id).first()
        if not user:
            raise ValueError("User not found")

        # Get user's learning history and performance
        user_progress = self._get_user_progress(user_id)
        performance_metrics = self._calculate_performance_metrics(user_id, user_progress)

        # Get content based on user's current level and needs
        recommended_content = self._get_content_recommendations(user_id, current_chapter_id, performance_metrics)

        return {
            "recommended_content": recommended_content,
            "performance_insights": performance_metrics,
            "learning_path_adjustments": self._get_learning_path_adjustments(user_id, performance_metrics),
            "support_resources": self._get_support_resources(user_id, performance_metrics)
        }

    def _get_user_progress(self, user_id: int) -> Dict[str, Any]:
        """
        Get user's learning progress across all content
        """
        # Get user's learning path progress
        learning_path_progress = self.db.query(UserLearningPathProgress).filter(
            UserLearningPathProgress.user_id == user_id
        ).all()

        # Get user's chapter progress
        # This would require a UserChapterProgress model if it doesn't exist
        # For now, we'll simulate with learning path progress

        # Get user's AI interaction history
        recent_interactions = self.db.query(AIChatMessage).join(AIChatSession).filter(
            AIChatSession.user_id == user_id,
            AIChatMessage.timestamp > datetime.utcnow() - timedelta(days=30)  # Last 30 days
        ).order_by(AIChatMessage.timestamp.desc()).limit(50).all()

        return {
            "learning_path_progress": learning_path_progress,
            "recent_interactions": recent_interactions,
            "total_interactions": len(recent_interactions)
        }

    def _calculate_performance_metrics(self, user_id: int, user_progress: Dict[str, Any]) -> Dict[str, Any]:
        """
        Calculate performance metrics based on user's activity
        """
        # Calculate completion rate
        completed_steps = len([p for p in user_progress['learning_path_progress'] if p.status == 'completed'])
        total_steps = len(user_progress['learning_path_progress'])

        completion_rate = (completed_steps / total_steps * 100) if total_steps > 0 else 0

        # Calculate engagement metrics
        recent_activity = len(user_progress['recent_interactions'])
        avg_interactions_per_day = recent_activity / 30 if recent_activity > 0 else 0

        # Calculate difficulty adaptation needs
        # Look for patterns where user might be struggling
        struggling_indicators = self._identify_struggling_patterns(user_progress)

        # Calculate learning style preferences based on interaction patterns
        learning_style = self._infer_learning_style(user_progress)

        return {
            "completion_rate": completion_rate,
            "engagement_score": avg_interactions_per_day * 10,  # Scale to 0-100
            "struggling_indicators": struggling_indicators,
            "learning_style": learning_style,
            "time_spent_learning": self._calculate_time_spent(user_progress),
            "preferred_difficulty": self._infer_difficulty_preference(user_progress)
        }

    def _identify_struggling_patterns(self, user_progress: Dict[str, Any]) -> List[str]:
        """
        Identify patterns that indicate user is struggling
        """
        indicators = []

        # Check for repeated questions about same topics
        topic_questions = {}
        for interaction in user_progress['recent_interactions']:
            # This would need to be enhanced with actual topic extraction
            if 'struggling' in interaction.content.lower() or 'help' in interaction.content.lower():
                indicators.append("Asking for help frequently")

        # Check for slow progress in learning paths
        if len(user_progress['learning_path_progress']) > 5:  # If user has started several steps
            recent_progress = user_progress['learning_path_progress'][:5]  # Last 5 steps
            slow_completion = sum(1 for p in recent_progress if p.status == 'completed') < 2
            if slow_completion:
                indicators.append("Slow completion rate")

        return indicators

    def _infer_learning_style(self, user_progress: Dict[str, Any]) -> str:
        """
        Infer user's learning style based on interaction patterns
        """
        # Analyze the types of questions asked
        theoretical_questions = 0
        practical_questions = 0

        for interaction in user_progress['recent_interactions']:
            content = interaction.content.lower()
            if any(word in content for word in ['theor', 'concept', 'principle', 'why']):
                theoretical_questions += 1
            if any(word in content for word in ['how', 'implement', 'code', 'example', 'practice']):
                practical_questions += 1

        if theoretical_questions > practical_questions * 1.5:
            return "theoretical"
        elif practical_questions > theoretical_questions * 1.5:
            return "practical"
        else:
            return "balanced"

    def _calculate_time_spent(self, user_progress: Dict[str, Any]) -> float:
        """
        Calculate estimated time spent learning
        """
        # This is a simplified calculation
        # In a real system, we would track actual time spent
        return len(user_progress['recent_interactions']) * 5  # Estimate 5 minutes per interaction

    def _infer_difficulty_preference(self, user_progress: Dict[str, Any]) -> str:
        """
        Infer preferred difficulty level based on progress
        """
        if not user_progress['learning_path_progress']:
            return "moderate"  # Default

        # Look at completion rates for different difficulty levels
        # This would need actual difficulty metadata
        return "moderate"

    def _get_content_recommendations(self, user_id: int, current_chapter_id: Optional[int],
                                   performance_metrics: Dict[str, Any]) -> List[Dict[str, Any]]:
        """
        Get content recommendations based on user's performance and needs
        """
        # Get all available chapters
        all_chapters = self.db.query(Chapter).filter(
            Chapter.is_published == True
        ).all()

        # Filter and rank chapters based on user's needs
        recommendations = []
        for chapter in all_chapters:
            score = self._calculate_adaptive_score(chapter, user_id, performance_metrics)
            if score > 0:
                recommendations.append({
                    'chapter': chapter,
                    'score': score,
                    'reason': self._get_recommendation_reason(chapter, performance_metrics)
                })

        # Sort by score
        recommendations.sort(key=lambda x: x['score'], reverse=True)

        # Format recommendations
        formatted_recs = []
        for rec in recommendations[:10]:  # Top 10 recommendations
            formatted_recs.append({
                'id': rec['chapter'].id,
                'title': rec['chapter'].title,
                'description': rec['chapter'].description,
                'reading_time_minutes': rec['chapter'].reading_time_minutes,
                'adaptation_score': rec['score'],
                'reason': rec['reason']
            })

        return formatted_recs

    def _calculate_adaptive_score(self, chapter: Chapter, user_id: int,
                                performance_metrics: Dict[str, Any]) -> float:
        """
        Calculate adaptation score for a chapter based on user's performance
        """
        score = 0.0

        # Adjust score based on user's struggling indicators
        if "Asking for help frequently" in performance_metrics.get('struggling_indicators', []):
            # If user is struggling, recommend more foundational content
            if 'basic' in chapter.title.lower() or 'introduction' in chapter.title.lower():
                score += 5.0
            elif 'advanced' in chapter.title.lower():
                score -= 3.0
        else:
            # If user is doing well, recommend more challenging content
            if 'advanced' in chapter.title.lower():
                score += 2.0

        # Adjust based on learning style
        if performance_metrics['learning_style'] == 'practical':
            if any(keyword in chapter.title.lower() for keyword in ['example', 'application', 'practice', 'exercise']):
                score += 3.0
        elif performance_metrics['learning_style'] == 'theoretical':
            if any(keyword in chapter.title.lower() for keyword in ['theory', 'principle', 'concept', 'foundation']):
                score += 3.0

        # Adjust based on completion rate
        if performance_metrics['completion_rate'] < 50:
            # Recommend shorter, easier content to build confidence
            if chapter.reading_time_minutes and chapter.reading_time_minutes < 30:
                score += 2.0
        elif performance_metrics['completion_rate'] > 80:
            # Recommend more challenging content for high performers
            score += 1.0

        # Boost score if it's a follow-up to current chapter (sequentially)
        # This would require knowledge of chapter relationships
        # For now, we'll just add a small boost for all chapters
        score += 1.0

        return max(0, score)  # Ensure non-negative score

    def _get_recommendation_reason(self, chapter: Chapter, performance_metrics: Dict[str, Any]) -> str:
        """
        Get the reason for recommending this chapter
        """
        reasons = []

        if "Asking for help frequently" in performance_metrics.get('struggling_indicators', []):
            if 'basic' in chapter.title.lower() or 'introduction' in chapter.title.lower():
                reasons.append("Based on your need for foundational concepts")

        if performance_metrics['learning_style'] == 'practical':
            if any(keyword in chapter.title.lower() for keyword in ['example', 'application', 'practice']):
                reasons.append("Matches your preference for practical examples")

        if performance_metrics['completion_rate'] < 50:
            if chapter.reading_time_minutes and chapter.reading_time_minutes < 30:
                reasons.append("Shorter content to build momentum")

        if reasons:
            return f"Recommended because: {', '.join(reasons)}"
        else:
            return "Recommended based on your learning pattern"

    def _get_learning_path_adjustments(self, user_id: int, performance_metrics: Dict[str, Any]) -> List[Dict[str, Any]]:
        """
        Get suggested adjustments to user's learning path based on performance
        """
        adjustments = []

        # If user is struggling, suggest slowing down or adding prerequisites
        if "Asking for help frequently" in performance_metrics.get('struggling_indicators', []):
            adjustments.append({
                "type": "difficulty_reduction",
                "message": "Consider reviewing foundational concepts before proceeding",
                "suggestion": "Review chapters on basic concepts"
            })

        # If user is excelling, suggest accelerating
        if performance_metrics['completion_rate'] > 90 and performance_metrics['engagement_score'] > 70:
            adjustments.append({
                "type": "acceleration",
                "message": "You're progressing well! Consider more challenging content",
                "suggestion": "Explore advanced topics"
            })

        # If engagement is low, suggest more interactive content
        if performance_metrics['engagement_score'] < 30:
            adjustments.append({
                "type": "engagement_boost",
                "message": "Try more interactive content to stay engaged",
                "suggestion": "Look for chapters with exercises and simulations"
            })

        return adjustments

    def _get_support_resources(self, user_id: int, performance_metrics: Dict[str, Any]) -> List[Dict[str, Any]]:
        """
        Get additional support resources based on user's needs
        """
        resources = []

        if "Asking for help frequently" in performance_metrics.get('struggling_indicators', []):
            resources.append({
                "type": "tutoring",
                "title": "Schedule a tutoring session",
                "description": "Get personalized help with challenging concepts",
                "priority": "high"
            })

        resources.append({
            "type": "study_group",
            "title": "Join a study group",
            "description": "Connect with other learners studying similar topics",
            "priority": "medium"
        })

        if performance_metrics['learning_style'] == 'practical':
            resources.append({
                "type": "hands_on",
                "title": "Additional practice exercises",
                "description": "More hands-on exercises to reinforce concepts",
                "priority": "medium"
            })

        return resources

    def get_personalized_reading_order(self, user_id: int, chapter_ids: List[int]) -> List[int]:
        """
        Get a personalized reading order for a set of chapters based on user's profile
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

        # Get chapters
        chapters = self.db.query(Chapter).filter(
            Chapter.id.in_(chapter_ids)
        ).all()

        # Create a list with scores
        scored_chapters = []
        for chapter in chapters:
            score = self._calculate_reading_order_score(chapter, professional_background, learning_preferences)
            scored_chapters.append((chapter.id, score))

        # Sort by score (descending) and return the order
        scored_chapters.sort(key=lambda x: x[1], reverse=True)
        return [chapter_id for chapter_id, score in scored_chapters]

    def _calculate_reading_order_score(self, chapter: Chapter, professional_background: str,
                                     learning_preferences: Dict[str, Any]) -> float:
        """
        Calculate score for reading order based on user profile
        """
        score = 0.0

        # Boost based on user's professional background
        chapter_content = f"{chapter.title} {chapter.description} {chapter.content}".lower()
        background_lower = professional_background.lower()

        if any(keyword in background_lower for keyword in ['robotics', 'engineer']):
            if any(keyword in chapter_content for keyword in ['robot', 'control', 'dynamics']):
                score += 5.0

        if any(keyword in background_lower for keyword in ['ai', 'ml', 'machine learning']):
            if any(keyword in chapter_content for keyword in ['ai', 'ml', 'neural']):
                score += 5.0

        # Adjust based on learning preferences
        if learning_preferences.get('preferred_pace') == 'slow':
            if chapter.reading_time_minutes and chapter.reading_time_minutes < 45:
                score += 2.0
        elif learning_preferences.get('preferred_pace') == 'fast':
            score += 1.0  # Fast-paced learners can handle longer content

        # Boost for content format preferences
        if 'theoretical' in learning_preferences.get('content_format', []):
            if any(keyword in chapter_content for keyword in ['theory', 'principle', 'mathematical']):
                score += 3.0

        if 'practical' in learning_preferences.get('content_format', []):
            if any(keyword in chapter_content for keyword in ['example', 'application', 'exercise']):
                score += 3.0

        return score


# Convenience function to get adaptive content service
def get_adaptive_content_service(db: Session) -> AdaptiveContentService:
    return AdaptiveContentService(db)
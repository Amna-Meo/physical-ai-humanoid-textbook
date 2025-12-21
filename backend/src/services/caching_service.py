from typing import Any, Optional, Dict
import hashlib
import json
import time
from datetime import datetime, timedelta
from functools import wraps
from sqlalchemy.orm import Session
import threading


class CacheEntry:
    """Represents a cached entry with expiration"""

    def __init__(self, value: Any, ttl_seconds: int):
        self.value = value
        self.created_at = datetime.utcnow()
        self.expires_at = self.created_at + timedelta(seconds=ttl_seconds)
        self.access_count = 0

    def is_expired(self) -> bool:
        """Check if the cache entry has expired"""
        return datetime.utcnow() > self.expires_at

    def increment_access(self):
        """Increment access count"""
        self.access_count += 1


class CachingService:
    """
    Service for caching mechanisms to improve performance
    """

    def __init__(self, default_ttl: int = 300):  # 5 minutes default TTL
        self.cache: Dict[str, CacheEntry] = {}
        self.default_ttl = default_ttl
        self.lock = threading.RLock()  # Thread-safe access

    def _generate_key(self, *args, **kwargs) -> str:
        """Generate a unique cache key from function arguments"""
        key_data = {
            'args': args,
            'kwargs': kwargs
        }
        key_str = json.dumps(key_data, sort_keys=True, default=str)
        return hashlib.md5(key_str.encode()).hexdigest()

    def get(self, key: str) -> Optional[Any]:
        """Get a value from cache"""
        with self.lock:
            if key in self.cache:
                entry = self.cache[key]
                if entry.is_expired():
                    del self.cache[key]
                    return None
                entry.increment_access()
                return entry.value
            return None

    def set(self, key: str, value: Any, ttl_seconds: Optional[int] = None) -> None:
        """Set a value in cache"""
        if ttl_seconds is None:
            ttl_seconds = self.default_ttl

        with self.lock:
            self.cache[key] = CacheEntry(value, ttl_seconds)

    def delete(self, key: str) -> bool:
        """Delete a value from cache"""
        with self.lock:
            if key in self.cache:
                del self.cache[key]
                return True
            return False

    def clear(self) -> None:
        """Clear all cache entries"""
        with self.lock:
            self.cache.clear()

    def get_stats(self) -> Dict[str, Any]:
        """Get cache statistics"""
        with self.lock:
            total_entries = len(self.cache)
            expired_entries = sum(1 for entry in self.cache.values() if entry.is_expired())
            active_entries = total_entries - expired_entries

            # Calculate average access count
            total_accesses = sum(entry.access_count for entry in self.cache.values() if not entry.is_expired())
            avg_accesses = total_accesses / active_entries if active_entries > 0 else 0

            return {
                "total_entries": total_entries,
                "active_entries": active_entries,
                "expired_entries": expired_entries,
                "average_accesses": round(avg_accesses, 2),
                "cache_keys": list(self.cache.keys())
            }

    def cache_result(self, ttl_seconds: Optional[int] = None):
        """Decorator to cache function results"""
        def decorator(func):
            @wraps(func)
            def wrapper(*args, **kwargs):
                key = self._generate_key(func.__name__, *args, **kwargs)

                # Try to get from cache
                cached_result = self.get(key)
                if cached_result is not None:
                    return cached_result

                # Execute function and cache result
                result = func(*args, **kwargs)
                self.set(key, result, ttl_seconds)
                return result

            return wrapper
        return decorator

    def cache_method(self, ttl_seconds: Optional[int] = None):
        """Decorator for caching method results (with self parameter)"""
        def decorator(func):
            @wraps(func)
            def wrapper(self_obj, *args, **kwargs):
                # Include the class and method name in the key
                class_name = self_obj.__class__.__name__
                method_name = func.__name__

                key = self._generate_key(class_name, method_name, *args, **kwargs)

                # Try to get from cache
                cached_result = self.get(key)
                if cached_result is not None:
                    return cached_result

                # Execute method and cache result
                result = func(self_obj, *args, **kwargs)
                self.set(key, result, ttl_seconds)
                return result

            return wrapper
        return decorator

    # Specific caching methods for common use cases

    def cache_chapter_content(self, chapter_id: int, content: str, ttl_seconds: int = 600):
        """Cache chapter content"""
        key = f"chapter_content_{chapter_id}"
        self.set(key, content, ttl_seconds)

    def get_cached_chapter_content(self, chapter_id: int) -> Optional[str]:
        """Get cached chapter content"""
        key = f"chapter_content_{chapter_id}"
        return self.get(key)

    def cache_user_learning_path(self, user_id: int, learning_path_data: Any, ttl_seconds: int = 300):
        """Cache user learning path data"""
        key = f"user_learning_path_{user_id}"
        self.set(key, learning_path_data, ttl_seconds)

    def get_cached_user_learning_path(self, user_id: int) -> Optional[Any]:
        """Get cached user learning path data"""
        key = f"user_learning_path_{user_id}"
        return self.get(key)

    def cache_ai_response(self, query_hash: str, response: str, ttl_seconds: int = 1800):
        """Cache AI responses to reduce API calls"""
        key = f"ai_response_{query_hash}"
        self.set(key, response, ttl_seconds)

    def get_cached_ai_response(self, query_hash: str) -> Optional[str]:
        """Get cached AI response"""
        key = f"ai_response_{query_hash}"
        return self.get(key)

    def cache_search_results(self, query: str, results: Any, ttl_seconds: int = 900):
        """Cache search results"""
        query_hash = hashlib.md5(query.encode()).hexdigest()
        key = f"search_results_{query_hash}"
        self.set(key, results, ttl_seconds)

    def get_cached_search_results(self, query: str) -> Optional[Any]:
        """Get cached search results"""
        query_hash = hashlib.md5(query.encode()).hexdigest()
        key = f"search_results_{query_hash}"
        return self.get(key)


# Global caching service instance
# In a real application, this would be configured via dependency injection
global_cache = CachingService(default_ttl=300)


def get_cache_service() -> CachingService:
    """Get the global caching service instance"""
    return global_cache


# Example usage with database session integration
class DatabaseCacheService:
    """
    Caching service that works with database sessions
    """

    def __init__(self, db: Session):
        self.db = db
        self.cache = get_cache_service()

    def get_chapter_with_cache(self, chapter_id: int):
        """Get chapter with caching"""
        # Try cache first
        cached_content = self.cache.get_cached_chapter_content(chapter_id)
        if cached_content:
            return cached_content

        # If not in cache, get from database
        from ..models.chapter import Chapter
        chapter = self.db.query(Chapter).filter(Chapter.id == chapter_id).first()

        if chapter:
            # Cache the result
            self.cache.cache_chapter_content(chapter_id, chapter.content)
            return chapter

        return None

    def get_user_learning_path_with_cache(self, user_id: int):
        """Get user learning path with caching"""
        # Try cache first
        cached_data = self.cache.get_cached_user_learning_path(user_id)
        if cached_data:
            return cached_data

        # If not in cache, get from database
        from ..services.learning_path_service import get_learning_path_service
        learning_path_service = get_learning_path_service(self.db)
        user_learning_paths = learning_path_service.get_user_learning_paths(user_id)

        if user_learning_paths:
            # Cache the result
            self.cache.cache_user_learning_path(user_id, user_learning_paths)
            return user_learning_paths

        return None


def get_database_cache_service(db: Session) -> DatabaseCacheService:
    return DatabaseCacheService(db)
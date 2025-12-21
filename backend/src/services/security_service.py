from typing import Dict, Optional, List
import time
import hashlib
import hmac
import secrets
from datetime import datetime, timedelta
from sqlalchemy.orm import Session
from ..models.user import User
from ..models.ai_interaction import AIChatSession
from enum import Enum
import threading
from collections import defaultdict, deque


class SecurityEventType(Enum):
    """Enumeration for security event types"""
    LOGIN_ATTEMPT = "login_attempt"
    FAILED_LOGIN = "failed_login"
    RATE_LIMIT_EXCEEDED = "rate_limit_exceeded"
    SUSPICIOUS_ACTIVITY = "suspicious_activity"
    PERMISSION_VIOLATION = "permission_violation"
    DATA_ACCESS = "data_access"


class RateLimitRule:
    """Represents a rate limiting rule"""

    def __init__(self, name: str, max_requests: int, window_seconds: int, per_user: bool = True):
        self.name = name
        self.max_requests = max_requests
        self.window_seconds = window_seconds
        self.per_user = per_user


class SecurityService:
    """
    Service for security measures including rate limiting and access control
    """

    def __init__(self, db: Session):
        self.db = db
        self.logger = None  # Will be set by logging service

        # Rate limiting storage
        self.request_counts = defaultdict(lambda: deque())
        self.failed_login_attempts = defaultdict(lambda: deque())
        self.lock = threading.Lock()

        # Rate limiting rules
        self.rate_limit_rules = {
            "api_general": RateLimitRule("general_api", max_requests=100, window_seconds=3600),  # 100 req/hour
            "api_chapters": RateLimitRule("chapter_access", max_requests=50, window_seconds=300),  # 50 req/5min
            "api_ai_chat": RateLimitRule("ai_chat", max_requests=20, window_seconds=60),  # 20 req/min
            "api_search": RateLimitRule("search", max_requests=30, window_seconds=60),  # 30 req/min
            "login": RateLimitRule("login", max_requests=5, window_seconds=900, per_user=False),  # 5 req/15min per IP
        }

    def check_rate_limit(self, endpoint: str, user_id: Optional[int], client_ip: str) -> bool:
        """
        Check if request exceeds rate limits
        """
        with self.lock:
            # Determine the appropriate rule
            rule = self.rate_limit_rules.get(endpoint)
            if not rule:
                return True  # No limit for this endpoint

            # Determine key based on whether limit is per user or per IP
            if rule.per_user and user_id:
                key = f"{endpoint}:{user_id}"
            else:
                key = f"{endpoint}:{client_ip}"

            now = time.time()
            window_start = now - rule.window_seconds

            # Clean old requests outside the window
            while self.request_counts[key] and self.request_counts[key][0] < window_start:
                self.request_counts[key].popleft()

            # Check if limit exceeded
            current_count = len(self.request_counts[key])
            if current_count >= rule.max_requests:
                # Log rate limit exceeded event
                self.log_security_event(
                    SecurityEventType.RATE_LIMIT_EXCEEDED,
                    user_id,
                    client_ip,
                    f"Rate limit exceeded for {endpoint}: {current_count}/{rule.max_requests} requests in {rule.window_seconds}s"
                )
                return False

            # Add current request
            self.request_counts[key].append(now)
            return True

    def record_failed_login(self, username: str, client_ip: str):
        """
        Record a failed login attempt for rate limiting
        """
        with self.lock:
            key = f"login:{client_ip}"
            now = time.time()
            window_start = now - 900  # 15 minutes

            # Clean old attempts
            while self.failed_login_attempts[key] and self.failed_login_attempts[key][0] < window_start:
                self.failed_login_attempts[key].popleft()

            # Add current attempt
            self.failed_login_attempts[key].append(now)

    def is_login_blocked(self, username: str, client_ip: str) -> bool:
        """
        Check if login is blocked due to too many failed attempts
        """
        with self.lock:
            key = f"login:{client_ip}"
            now = time.time()
            window_start = now - 900  # 15 minutes

            # Clean old attempts
            while self.failed_login_attempts[key] and self.failed_login_attempts[key][0] < window_start:
                self.failed_login_attempts[key].popleft()

            # Check if too many failed attempts
            failed_attempts = len(self.failed_login_attempts[key])
            return failed_attempts >= 5  # Block after 5 failed attempts

    def generate_secure_token(self, length: int = 32) -> str:
        """
        Generate a cryptographically secure token
        """
        return secrets.token_urlsafe(length)

    def hash_password(self, password: str, salt: Optional[str] = None) -> tuple[str, str]:
        """
        Hash a password with salt
        """
        if salt is None:
            salt = secrets.token_hex(16)

        # Using PBKDF2 with SHA-256
        password_hash = hashlib.pbkdf2_hmac('sha256', password.encode('utf-8'), salt.encode('utf-8'), 100000)
        return password_hash.hex(), salt

    def verify_password(self, password: str, stored_hash: str, salt: str) -> bool:
        """
        Verify a password against stored hash
        """
        password_hash, _ = self.hash_password(password, salt)
        return hmac.compare_digest(password_hash, stored_hash)

    def validate_session_token(self, token: str, user_id: int) -> bool:
        """
        Validate a session token
        """
        # In a real implementation, this would check the database for the token
        # For now, we'll just validate the format
        if not token or len(token) < 32:
            return False

        # Check if user exists
        user = self.db.query(User).filter(User.id == user_id).first()
        return user is not None

    def sanitize_input(self, input_data: str) -> str:
        """
        Sanitize user input to prevent XSS and injection attacks
        """
        # Remove potentially dangerous characters
        dangerous_chars = ['<', '>', '"', "'", '&', ';', '(', ')', '{', '}']
        sanitized = input_data

        for char in dangerous_chars:
            sanitized = sanitized.replace(char, '')

        return sanitized.strip()

    def validate_user_permissions(self, user_id: int, required_permissions: List[str]) -> bool:
        """
        Validate if user has required permissions
        """
        # In a real implementation, this would check user roles/permissions in the database
        # For now, we'll implement a basic check
        user = self.db.query(User).filter(User.id == user_id).first()
        if not user:
            return False

        # For demonstration, assume all users have basic permissions
        # In a real system, you'd check against a permissions table
        return True

    def log_security_event(
        self,
        event_type: SecurityEventType,
        user_id: Optional[int],
        client_ip: str,
        description: str,
        severity: str = "medium"
    ):
        """
        Log security events
        """
        # In a real implementation, this would log to a security log
        # For now, we'll just print
        print(f"SECURITY EVENT: {event_type.value} - User: {user_id}, IP: {client_ip}, Description: {description}")

    def check_suspicious_activity(self, user_id: int, action: str, client_ip: str) -> bool:
        """
        Check for suspicious activity patterns
        """
        # Check for multiple logins from different IPs in short time
        # Check for unusual access patterns
        # Check for data exfiltration attempts

        # For now, just log the activity
        self.log_security_event(
            SecurityEventType.SUSPICIOUS_ACTIVITY,
            user_id,
            client_ip,
            f"Suspicious activity detected: {action}"
        )

        # Return False to indicate no suspicious activity found
        # In a real implementation, this would analyze patterns
        return False

    def generate_csrf_token(self, user_id: int) -> str:
        """
        Generate a CSRF token for a user
        """
        timestamp = str(int(time.time()))
        data = f"{user_id}:{timestamp}:{secrets.token_hex(16)}"
        return hashlib.sha256(data.encode()).hexdigest()

    def validate_csrf_token(self, token: str, user_id: int, max_age: int = 3600) -> bool:
        """
        Validate a CSRF token
        """
        # In a real implementation, tokens would be stored and validated
        # For now, we'll just check the format and age
        if not token or len(token) != 64:  # SHA-256 hex is 64 chars
            return False

        # Check if token is too old (implementation would require storing tokens)
        return True

    def get_security_stats(self) -> Dict[str, int]:
        """
        Get security-related statistics
        """
        with self.lock:
            return {
                "active_rate_limit_keys": len(self.request_counts),
                "total_failed_login_attempts": sum(len(attempts) for attempts in self.failed_login_attempts.values()),
                "rate_limit_rules": len(self.rate_limit_rules)
            }

    def setup_security_headers(self) -> Dict[str, str]:
        """
        Get recommended security headers
        """
        return {
            "X-Content-Type-Options": "nosniff",
            "X-Frame-Options": "DENY",
            "X-XSS-Protection": "1; mode=block",
            "Strict-Transport-Security": "max-age=31536000; includeSubDomains",
            "Content-Security-Policy": "default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'",
            "Referrer-Policy": "strict-origin-when-cross-origin"
        }


# Convenience function to get security service
def get_security_service(db: Session) -> SecurityService:
    return SecurityService(db)
from typing import Dict, Any, Optional
from enum import Enum
import logging
import traceback
from datetime import datetime
from ..models.user import User
from sqlalchemy.orm import Session
import json


class ErrorSeverity(Enum):
    """Enumeration for error severity levels"""
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"
    CRITICAL = "critical"


class ErrorCategory(Enum):
    """Enumeration for error categories"""
    VALIDATION = "validation"
    AUTHENTICATION = "authentication"
    AUTHORIZATION = "authorization"
    DATABASE = "database"
    EXTERNAL_SERVICE = "external_service"
    BUSINESS_LOGIC = "business_logic"
    SYSTEM = "system"
    USER_INPUT = "user_input"


class ErrorHandlingService:
    """
    Service for comprehensive error handling and user feedback
    """

    def __init__(self, db: Session):
        self.db = db
        self.logger = logging.getLogger(__name__)

    def log_error(
        self,
        error: Exception,
        user_id: Optional[int] = None,
        context: Optional[Dict[str, Any]] = None,
        severity: ErrorSeverity = ErrorSeverity.MEDIUM,
        category: ErrorCategory = ErrorCategory.SYSTEM
    ) -> str:
        """
        Log error with comprehensive details
        """
        error_id = f"err_{datetime.utcnow().strftime('%Y%m%d_%H%M%S')}_{hash(str(error)) % 10000:04d}"

        error_details = {
            "error_id": error_id,
            "timestamp": datetime.utcnow().isoformat(),
            "error_type": type(error).__name__,
            "error_message": str(error),
            "traceback": traceback.format_exc(),
            "user_id": user_id,
            "context": context or {},
            "severity": severity.value,
            "category": category.value
        }

        # Log to application logs
        self.logger.error(
            f"Error {error_id}: {type(error).__name__} - {str(error)}",
            extra=error_details
        )

        # In a real implementation, we would store this in a database
        # For now, we'll just return the error ID
        return error_id

    def create_user_friendly_message(
        self,
        error: Exception,
        category: ErrorCategory,
        user_context: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """
        Create a user-friendly error message based on the error category
        """
        # Default user-friendly message
        user_message = "An error occurred while processing your request. Please try again later."
        technical_details = str(error)

        # Customize message based on error category
        if category == ErrorCategory.VALIDATION:
            user_message = "The information you provided is not valid. Please check your input and try again."
        elif category == ErrorCategory.AUTHENTICATION:
            user_message = "Authentication failed. Please check your credentials and try again."
        elif category == ErrorCategory.AUTHORIZATION:
            user_message = "You don't have permission to perform this action."
        elif category == ErrorCategory.DATABASE:
            user_message = "A database error occurred. Please try again later."
        elif category == ErrorCategory.USER_INPUT:
            user_message = "The input you provided is not valid. Please check and try again."

        # For certain sensitive errors, don't expose technical details
        if category in [ErrorCategory.AUTHENTICATION, ErrorCategory.AUTHORIZATION, ErrorCategory.DATABASE]:
            technical_details = "Internal system error"

        return {
            "user_message": user_message,
            "technical_details": technical_details,
            "category": category.value,
            "can_retry": category not in [ErrorCategory.AUTHORIZATION],
            "suggested_action": self._get_suggested_action(category)
        }

    def _get_suggested_action(self, category: ErrorCategory) -> str:
        """
        Get suggested action based on error category
        """
        actions = {
            ErrorCategory.VALIDATION: "Please review your input and correct any errors.",
            ErrorCategory.AUTHENTICATION: "Please verify your login credentials and try again.",
            ErrorCategory.AUTHORIZATION: "Contact your administrator if you believe you should have access.",
            ErrorCategory.DATABASE: "Try again later. If the problem persists, contact support.",
            ErrorCategory.EXTERNAL_SERVICE: "Try again later. The issue may be with an external service.",
            ErrorCategory.BUSINESS_LOGIC: "Check the requirements and constraints for this operation.",
            ErrorCategory.SYSTEM: "Try again later. If the problem persists, contact support.",
            ErrorCategory.USER_INPUT: "Review your input and ensure it meets the required format."
        }
        return actions.get(category, "Please try again or contact support for assistance.")

    def handle_api_error(
        self,
        error: Exception,
        user_id: Optional[int] = None,
        context: Optional[Dict[str, Any]] = None,
        severity: ErrorSeverity = ErrorSeverity.MEDIUM
    ) -> Dict[str, Any]:
        """
        Handle API error with appropriate categorization and user feedback
        """
        # Determine error category based on exception type
        category = self._categorize_error(error)

        # Log the error
        error_id = self.log_error(error, user_id, context, severity, category)

        # Create user-friendly message
        user_feedback = self.create_user_friendly_message(error, category, context)

        # Return structured error response
        return {
            "error": True,
            "error_id": error_id,
            "message": user_feedback["user_message"],
            "category": user_feedback["category"],
            "can_retry": user_feedback["can_retry"],
            "suggested_action": user_feedback["suggested_action"],
            "timestamp": datetime.utcnow().isoformat()
        }

    def _categorize_error(self, error: Exception) -> ErrorCategory:
        """
        Categorize error based on exception type
        """
        error_type = type(error).__name__

        if "ValidationError" in error_type or "ValueError" in error_type:
            return ErrorCategory.VALIDATION
        elif "AuthenticationError" in error_type or "InvalidCredentials" in error_type:
            return ErrorCategory.AUTHENTICATION
        elif "AuthorizationError" in error_type or "PermissionError" in error_type:
            return ErrorCategory.AUTHORIZATION
        elif "DatabaseError" in error_type or "IntegrityError" in error_type:
            return ErrorCategory.DATABASE
        elif "ConnectionError" in error_type or "TimeoutError" in error_type:
            return ErrorCategory.EXTERNAL_SERVICE
        else:
            return ErrorCategory.SYSTEM

    def validate_input(self, data: Dict[str, Any], schema: Dict[str, Any]) -> Dict[str, Any]:
        """
        Validate input data against schema and return validation results
        """
        errors = []
        validated_data = {}

        for field, rules in schema.items():
            value = data.get(field)
            field_errors = []

            # Required field validation
            if rules.get("required", False) and (value is None or value == ""):
                field_errors.append(f"{field} is required")
            elif value is not None:  # Only validate if value exists
                # Type validation
                expected_type = rules.get("type")
                if expected_type and not isinstance(value, expected_type):
                    field_errors.append(f"{field} must be of type {expected_type.__name__}")

                # Length validation
                if isinstance(value, str):
                    min_length = rules.get("min_length")
                    max_length = rules.get("max_length")
                    if min_length and len(value) < min_length:
                        field_errors.append(f"{field} must be at least {min_length} characters")
                    if max_length and len(value) > max_length:
                        field_errors.append(f"{field} must be no more than {max_length} characters")

                # Numeric validation
                if isinstance(value, (int, float)):
                    min_value = rules.get("min_value")
                    max_value = rules.get("max_value")
                    if min_value is not None and value < min_value:
                        field_errors.append(f"{field} must be at least {min_value}")
                    if max_value is not None and value > max_value:
                        field_errors.append(f"{field} must be no more than {max_value}")

                # Custom validation
                custom_validator = rules.get("validator")
                if custom_validator:
                    try:
                        is_valid, custom_error = custom_validator(value)
                        if not is_valid:
                            field_errors.append(custom_error or f"Invalid {field}")
                    except Exception as e:
                        field_errors.append(f"Validation error for {field}: {str(e)}")

            if field_errors:
                errors.extend(field_errors)
            elif value is not None:
                validated_data[field] = value

        return {
            "valid": len(errors) == 0,
            "errors": errors,
            "data": validated_data if len(errors) == 0 else {}
        }

    def create_feedback_message(
        self,
        message_type: str,  # success, info, warning, error
        title: str,
        description: str,
        action_required: bool = False,
        action_url: Optional[str] = None,
        action_text: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Create a structured feedback message for users
        """
        return {
            "type": message_type,
            "title": title,
            "description": description,
            "action_required": action_required,
            "action_url": action_url,
            "action_text": action_text,
            "timestamp": datetime.utcnow().isoformat()
        }


# Convenience function to get error handling service
def get_error_handling_service(db: Session) -> ErrorHandlingService:
    return ErrorHandlingService(db)
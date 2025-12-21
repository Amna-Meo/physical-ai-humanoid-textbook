from typing import Dict, Any, Optional, List
import logging
from datetime import datetime
import json
from sqlalchemy.orm import Session
from ..models.user import User
from ..models.chapter import Chapter
import time
from enum import Enum


class LogEventType(Enum):
    """Enumeration for different types of log events"""
    USER_ACTION = "user_action"
    SYSTEM_EVENT = "system_event"
    API_CALL = "api_call"
    ERROR = "error"
    SECURITY = "security"
    PERFORMANCE = "performance"
    DATA_ACCESS = "data_access"


class LogLevel(Enum):
    """Enumeration for log levels"""
    DEBUG = "debug"
    INFO = "info"
    WARNING = "warning"
    ERROR = "error"
    CRITICAL = "critical"


class LoggingService:
    """
    Service for comprehensive logging and monitoring
    """

    def __init__(self, db: Session):
        self.db = db
        self.logger = logging.getLogger(__name__)
        # Performance tracking
        self.performance_metrics = {}

    def log_event(
        self,
        event_type: LogEventType,
        level: LogLevel,
        message: str,
        user_id: Optional[int] = None,
        context: Optional[Dict[str, Any]] = None,
        api_endpoint: Optional[str] = None,
        request_id: Optional[str] = None
    ) -> str:
        """
        Log an event with comprehensive details
        """
        log_id = f"log_{datetime.utcnow().strftime('%Y%m%d_%H%M%S')}_{hash(message) % 10000:04d}"

        log_entry = {
            "log_id": log_id,
            "timestamp": datetime.utcnow().isoformat(),
            "event_type": event_type.value,
            "level": level.value,
            "message": message,
            "user_id": user_id,
            "context": context or {},
            "api_endpoint": api_endpoint,
            "request_id": request_id
        }

        # Log to application logs
        getattr(self.logger, level.value)(message, extra=log_entry)

        # In a real implementation, we would store this in a database
        # For now, we'll just return the log ID
        return log_id

    def log_api_call(
        self,
        endpoint: str,
        method: str,
        user_id: Optional[int],
        response_time: float,
        status_code: int,
        request_data: Optional[Dict[str, Any]] = None,
        response_data: Optional[Dict[str, Any]] = None
    ) -> str:
        """
        Log API call details for monitoring
        """
        context = {
            "method": method,
            "response_time_ms": round(response_time * 1000, 2),
            "status_code": status_code,
            "request_size": len(json.dumps(request_data or {})),
            "response_size": len(json.dumps(response_data or {}))
        }

        message = f"API call to {endpoint} completed with status {status_code} in {response_time:.3f}s"

        # Determine event type based on status code
        if status_code >= 400:
            event_type = LogEventType.ERROR
            level = LogLevel.WARNING if status_code < 500 else LogLevel.ERROR
        else:
            event_type = LogEventType.API_CALL
            level = LogLevel.INFO

        return self.log_event(
            event_type=event_type,
            level=level,
            message=message,
            user_id=user_id,
            context=context,
            api_endpoint=endpoint
        )

    def log_performance_metric(
        self,
        metric_name: str,
        value: float,
        unit: str,
        user_id: Optional[int] = None,
        context: Optional[Dict[str, Any]] = None
    ) -> str:
        """
        Log performance metrics
        """
        if metric_name not in self.performance_metrics:
            self.performance_metrics[metric_name] = []

        self.performance_metrics[metric_name].append({
            "timestamp": datetime.utcnow(),
            "value": value,
            "unit": unit
        })

        context = context or {}
        context.update({
            "metric_name": metric_name,
            "value": value,
            "unit": unit
        })

        message = f"Performance metric: {metric_name} = {value} {unit}"

        return self.log_event(
            event_type=LogEventType.PERFORMANCE,
            level=LogLevel.INFO,
            message=message,
            user_id=user_id,
            context=context
        )

    def log_security_event(
        self,
        event_type: str,
        user_id: Optional[int],
        ip_address: Optional[str],
        description: str,
        severity: str = "medium"  # low, medium, high, critical
    ) -> str:
        """
        Log security-related events
        """
        context = {
            "security_event_type": event_type,
            "ip_address": ip_address,
            "severity": severity
        }

        message = f"Security event: {event_type} - {description}"

        return self.log_event(
            event_type=LogEventType.SECURITY,
            level=LogLevel.WARNING if severity in ["high", "critical"] else LogLevel.INFO,
            message=message,
            user_id=user_id,
            context=context
        )

    def log_user_action(
        self,
        user_id: int,
        action: str,
        target_resource: Optional[str] = None,
        target_id: Optional[int] = None
    ) -> str:
        """
        Log user actions for audit trail
        """
        context = {
            "action": action,
            "target_resource": target_resource,
            "target_id": target_id
        }

        message = f"User {user_id} performed action: {action}"
        if target_resource:
            message += f" on {target_resource} {target_id or ''}"

        return self.log_event(
            event_type=LogEventType.USER_ACTION,
            level=LogLevel.INFO,
            message=message,
            user_id=user_id,
            context=context
        )

    def get_performance_summary(self) -> Dict[str, Any]:
        """
        Get performance summary for monitoring
        """
        summary = {}

        for metric_name, values in self.performance_metrics.items():
            if values:
                metric_values = [v["value"] for v in values]
                summary[metric_name] = {
                    "count": len(metric_values),
                    "average": sum(metric_values) / len(metric_values),
                    "min": min(metric_values),
                    "max": max(metric_values),
                    "last_10_values": metric_values[-10:]  # Last 10 values
                }

        return {
            "timestamp": datetime.utcnow().isoformat(),
            "performance_metrics": summary,
            "total_logged_metrics": len(self.performance_metrics)
        }

    def monitor_api_performance(
        self,
        endpoint: str,
        method: str,
        user_id: Optional[int],
        func,
        *args,
        **kwargs
    ):
        """
        Monitor API performance by wrapping function calls
        """
        start_time = time.time()
        request_id = f"req_{datetime.utcnow().strftime('%Y%m%d_%H%M%S')}_{hash(str(args) + str(kwargs)) % 10000:04d}"

        try:
            result = func(*args, **kwargs)
            response_time = time.time() - start_time

            # Log successful API call
            log_id = self.log_api_call(
                endpoint=endpoint,
                method=method,
                user_id=user_id,
                response_time=response_time,
                status_code=200,
                request_data=kwargs.get('request_data'),
                response_data=kwargs.get('response_data')
            )

            # Log performance metric
            self.log_performance_metric(
                metric_name=f"api_response_time_{endpoint.replace('/', '_')}",
                value=response_time,
                unit="seconds",
                user_id=user_id
            )

            return result

        except Exception as e:
            response_time = time.time() - start_time

            # Log failed API call
            log_id = self.log_api_call(
                endpoint=endpoint,
                method=method,
                user_id=user_id,
                response_time=response_time,
                status_code=500,
                request_data=kwargs.get('request_data')
            )

            # Log the error
            from .error_handling_service import ErrorHandlingService, ErrorCategory
            error_service = ErrorHandlingService(self.db)
            error_service.log_error(
                e,
                user_id=user_id,
                context={"endpoint": endpoint, "method": method, "response_time": response_time},
                category=ErrorCategory.SYSTEM
            )

            raise

    def get_audit_trail(
        self,
        user_id: Optional[int] = None,
        event_type: Optional[LogEventType] = None,
        start_date: Optional[datetime] = None,
        end_date: Optional[datetime] = None,
        limit: int = 100
    ) -> List[Dict[str, Any]]:
        """
        Get audit trail for compliance and monitoring
        This is a simplified implementation - in real system would query database
        """
        # In a real implementation, this would query a logging database
        # For now, we'll return an empty list with a note
        return [
            {
                "message": "Audit trail functionality implemented. In a production system, this would return actual log entries from the database.",
                "note": "This is a placeholder implementation. Actual audit trail would be retrieved from persistent storage.",
                "filters_applied": {
                    "user_id": user_id,
                    "event_type": event_type.value if event_type else None,
                    "start_date": start_date.isoformat() if start_date else None,
                    "end_date": end_date.isoformat() if end_date else None,
                    "limit": limit
                }
            }
        ]

    def setup_logging_config(self):
        """
        Setup logging configuration for the application
        """
        # Configure root logger
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.StreamHandler(),  # Console output
                # In production, you'd also add file handlers
            ]
        )

        # Set specific loggers to appropriate levels
        logging.getLogger('sqlalchemy.engine').setLevel(logging.WARNING)
        logging.getLogger('urllib3').setLevel(logging.WARNING)


# Convenience function to get logging service
def get_logging_service(db: Session) -> LoggingService:
    return LoggingService(db)
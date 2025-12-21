from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
import os
from dotenv import load_dotenv
import logging
import traceback
from datetime import datetime

# Load environment variables
load_dotenv()

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI(
    title="Physical AI & Humanoid Robotics Textbook API",
    description="Backend API for the Physical AI & Humanoid Robotics Textbook platform",
    version="0.1.0"
)

# Add CORS middleware
# TODO: In production, replace ["*"] with specific origins
allowed_origins = os.getenv("ALLOWED_ORIGINS", "*").split(",")
app.add_middleware(
    CORSMiddleware,
    allow_origins=allowed_origins,
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allow_headers=["*"],
)

# Global exception handler
@app.exception_handler(Exception)
async def global_exception_handler(request: Request, exc: Exception):
    """
    Global exception handler for uncaught exceptions
    """
    error_id = f"global_{datetime.utcnow().strftime('%Y%m%d_%H%M%S')}_{hash(str(exc)) % 10000:04d}"

    # Log the error with full traceback
    logger.error(
        f"Global exception {error_id}: {type(exc).__name__} - {str(exc)}",
        extra={
            "error_id": error_id,
            "url": str(request.url),
            "method": request.method,
            "headers": dict(request.headers),
            "timestamp": datetime.utcnow().isoformat(),
            "traceback": traceback.format_exc()
        }
    )

    # Return user-friendly error response
    return JSONResponse(
        status_code=500,
        content={
            "error": True,
            "error_id": error_id,
            "message": "An unexpected error occurred. Please try again later.",
            "category": "system",
            "can_retry": True,
            "suggested_action": "Try again later. If the problem persists, contact support.",
            "timestamp": datetime.utcnow().isoformat()
        }
    )

# HTTP Exception handler for FastAPI HTTPException
@app.exception_handler(404)
async def not_found_handler(request: Request, exc):
    return JSONResponse(
        status_code=404,
        content={
            "error": True,
            "message": "The requested resource was not found.",
            "category": "not_found",
            "can_retry": False,
            "suggested_action": "Check the URL and try again."
        }
    )

@app.exception_handler(422)
async def validation_error_handler(request: Request, exc):
    return JSONResponse(
        status_code=422,
        content={
            "error": True,
            "message": "Validation error in request data.",
            "category": "validation",
            "can_retry": True,
            "suggested_action": "Check the request data format and try again.",
            "details": str(exc)
        }
    )

@app.get("/")
async def root():
    return {"message": "Welcome to the Physical AI & Humanoid Robotics Textbook API"}

@app.get("/health")
async def health_check():
    return {"status": "healthy"}

# Include API routes
from . import user_routes
from . import ai_routes
from . import chapter_routes
from . import course_routes
app.include_router(user_routes.router, prefix="/api/v1/users", tags=["users"])
app.include_router(ai_routes.router, prefix="/api/v1/ai", tags=["ai"])
app.include_router(chapter_routes.router, prefix="/api/v1/chapters", tags=["chapters"])
app.include_router(course_routes.router, prefix="/api/v1/courses", tags=["courses"])

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
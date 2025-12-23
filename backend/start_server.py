"""
Production startup script for Physical AI & Humanoid Robotics Textbook backend
"""

import os
import sys
import subprocess
import logging
from pathlib import Path

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

def run_migrations():
    """Run database migrations"""
    logger.info("Running database migrations...")
    try:
        # Change to backend directory
        backend_dir = Path(__file__).parent
        os.chdir(backend_dir)
        
        # Run alembic migrations
        result = subprocess.run([
            sys.executable, "-m", "alembic", "upgrade", "head"
        ], capture_output=True, text=True, cwd=backend_dir)
        
        if result.returncode != 0:
            logger.error(f"Migration failed: {result.stderr}")
            raise Exception(f"Migration failed: {result.stderr}")
        
        logger.info("Database migrations completed successfully")
    except Exception as e:
        logger.error(f"Error running migrations: {str(e)}")
        raise

def start_server():
    """Start the Uvicorn server with production settings"""
    import uvicorn
    
    # Import the app
    sys.path.insert(0, str(Path(__file__).parent))
    from src.api.main import app
    
    # Get configuration from environment
    host = os.getenv("HOST", "0.0.0.0")
    port = int(os.getenv("PORT", "8000"))
    workers = int(os.getenv("WORKERS", "1"))
    
    logger.info(f"Starting server on {host}:{port} with {workers} worker(s)")
    
    # Start the server
    uvicorn.run(
        app,
        host=host,
        port=port,
        workers=workers,
        log_level=os.getenv("LOG_LEVEL", "info"),
        access_log=True,
        forwarded_allow_ips="*",
        proxy_headers=True
    )

def main():
    """Main entry point"""
    logger.info("Starting Physical AI Textbook backend...")
    
    # Run migrations first
    try:
        run_migrations()
    except Exception as e:
        logger.error(f"Failed to run migrations: {str(e)}")
        sys.exit(1)
    
    # Start the server
    try:
        start_server()
    except KeyboardInterrupt:
        logger.info("Server shutdown requested")
    except Exception as e:
        logger.error(f"Server error: {str(e)}")
        sys.exit(1)

if __name__ == "__main__":
    main()
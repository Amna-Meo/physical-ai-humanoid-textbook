# Physical AI & Humanoid Robotics Textbook - Environment Variables Documentation

## Database Configuration
DATABASE_URL=postgresql://textbook_user:your_secure_password@db:5432/physical_ai_textbook
DATABASE_NAME=physical_ai_textbook
DATABASE_USER=textbook_user
DATABASE_PASSWORD=your_secure_password

## AI Service Keys (Never commit these to version control)
GEMINI_API_KEY=your_gemini_api_key_here
OPENAI_API_KEY=your_openai_api_key_here

## Vector Database Configuration
QDRANT_URL=http://qdrant:6333
QDRANT_API_KEY=your_qdrant_api_key_here

## Security Configuration
SECRET_KEY=your_production_secret_key_here_must_be_32_chars_at_least
ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30

## Application Configuration
DEBUG=false
PROJECT_NAME=Physical AI Textbook
API_V1_STR=/api/v1

## CORS Configuration
ALLOWED_ORIGINS=https://yourdomain.com,https://www.yourdomain.com

## Domain Configuration
DOMAIN_NAME=yourdomain.com

## Logging Configuration
LOG_LEVEL=INFO
LOG_FILE_PATH=/app/logs/app.log

## Performance Configuration
MAX_WORKERS=4
WORKER_CLASS=uvicorn.workers.UvicornWorker

## Redis Configuration (if used)
REDIS_URL=redis://redis:6379/0

## File Upload Configuration
MAX_UPLOAD_SIZE=50MB
UPLOAD_DIR=/app/uploads
ALLOWED_FILE_TYPES=.pdf,.doc,.docx,.txt,.jpg,.jpeg,.png
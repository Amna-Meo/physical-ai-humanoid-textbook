# Physical AI & Humanoid Robotics Textbook - Deployment Guide

## Table of Contents
1. [Overview](#overview)
2. [Prerequisites](#prerequisites)
3. [Environment Setup](#environment-setup)
4. [Backend Deployment](#backend-deployment)
5. [Frontend Deployment](#frontend-deployment)
6. [Database Setup](#database-setup)
7. [Configuration](#configuration)
8. [Testing](#testing)
9. [Go-Live Checklist](#go-live-checklist)
10. [Post-Deployment](#post-deployment)

## Overview

This document provides a comprehensive guide for deploying the Physical AI & Humanoid Robotics Textbook platform. The platform consists of a FastAPI backend, a React frontend, and various supporting services for AI integration, content management, and user management.

## Prerequisites

### System Requirements
- **Operating System**: Linux (Ubuntu 20.04+), macOS, or Windows with WSL2
- **Python**: 3.11+
- **Node.js**: 18+ with npm 8+
- **Docker**: 20.10+ (optional but recommended)
- **Git**: 2.30+
- **Database**: PostgreSQL 13+ or SQLite (for development)

### External Services
- **Vector Database**: Qdrant or similar for RAG functionality
- **AI Provider**: OpenAI API key for AI chat functionality
- **Cloud Storage**: For file uploads (optional)

## Environment Setup

### 1. Clone the Repository
```bash
git clone https://github.com/your-organization/physical-ai-textbook.git
cd physical-ai-textbook
```

### 2. Backend Setup
```bash
# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install backend dependencies
pip install -r requirements.txt
```

### 3. Frontend Setup
```bash
cd frontend
npm install
cd ..
```

## Backend Deployment

### 1. Environment Variables
Create a `.env` file in the project root:

```env
# Database
DATABASE_URL=postgresql://username:password@localhost/physical_ai_textbook

# AI Services
OPENAI_API_KEY=your_openai_api_key_here

# Vector Database
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key

# Security
SECRET_KEY=your_secret_key_here
ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30

# Application
DEBUG=False
PROJECT_NAME=Physical AI Textbook
API_V1_STR=/api/v1
```

### 2. Database Migration
```bash
# Run database migrations
cd backend
alembic upgrade head
cd ..
```

### 3. Start Backend Server
```bash
# Using uvicorn
cd backend/src
uvicorn api.main:app --host 0.0.0.0 --port 8000 --reload
```

## Frontend Deployment

### 1. Build Frontend
```bash
cd frontend
npm run build
```

### 2. Environment Configuration
Update `frontend/.env.production`:

```env
REACT_APP_API_URL=https://your-domain.com/api/v1
REACT_APP_DEBUG=false
```

### 3. Serve Frontend
The frontend can be served through various methods:

#### Option A: Static File Server
```bash
# Serve build directory
npx serve -s build
```

#### Option B: Nginx Configuration
```nginx
server {
    listen 80;
    server_name your-domain.com;

    location / {
        root /path/to/physical-ai-textbook/frontend/build;
        try_files $uri $uri/ /index.html;
    }

    location /api/ {
        proxy_pass http://localhost:8000/;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
    }
}
```

## Database Setup

### 1. PostgreSQL Setup (Recommended for Production)
```sql
-- Create database
CREATE DATABASE physical_ai_textbook;

-- Create user
CREATE USER textbook_user WITH PASSWORD 'secure_password';

-- Grant privileges
GRANT ALL PRIVILEGES ON DATABASE physical_ai_textbook TO textbook_user;
```

### 2. Initial Data Population
```bash
# Run initial data scripts if available
python scripts/populate_initial_data.py
```

## Configuration

### 1. Security Configuration
- Set strong `SECRET_KEY` for JWT tokens
- Configure HTTPS with SSL certificate
- Set up proper CORS origins
- Enable security headers

### 2. Performance Configuration
- Configure database connection pooling
- Set up Redis for caching (optional)
- Configure CDN for static assets
- Set up load balancer (for high-traffic deployments)

### 3. Monitoring Configuration
- Set up logging to external service
- Configure health check endpoints
- Set up performance monitoring
- Configure error tracking

## Testing

### 1. Pre-Deployment Tests
```bash
# Run backend tests
cd backend
python -m pytest tests/

# Run frontend tests
cd frontend
npm test

# Run end-to-end tests
cd ..
python test_end_to_end.py
```

### 2. Load Testing
```bash
# Using the performance testing service
python -c "
from backend.src.services.performance_testing_service import get_performance_testing_service
from backend.src.lib.database import get_db
db = next(get_db())
perf_service = get_performance_testing_service(db)
results = perf_service.run_endpoint_performance_test(['/api/chapters', '/api/users/me'])
print('Performance Test Results:', results)
"
```

### 3. Security Testing
```bash
# Run security tests
python -c "
from backend.src.services.security_testing_service import get_security_testing_service
from backend.src.lib.database import get_db
db = next(get_db())
security_service = get_security_testing_service(db)
report = security_service.generate_security_report()
print('Security Report:', report)
"
```

## Go-Live Checklist

### ✅ Pre-Launch Verification
- [ ] All tests pass (unit, integration, end-to-end)
- [ ] Security scan completed and issues addressed
- [ ] Performance benchmarks met
- [ ] Database migration scripts tested
- [ ] Backup and recovery procedures verified
- [ ] SSL certificate installed and working
- [ ] DNS configuration updated
- [ ] Monitoring and alerting configured

### ✅ Application Verification
- [ ] Backend API endpoints accessible
- [ ] Frontend loads correctly
- [ ] User registration/login works
- [ ] Chapter content accessible
- [ ] AI chat functionality working
- [ ] Learning paths and progress tracking functional
- [ ] Course management features working
- [ ] Content recommendation engine operational

### ✅ Infrastructure Verification
- [ ] Server resources adequate for expected load
- [ ] Database connections stable
- [ ] Vector database connectivity verified
- [ ] AI API keys working correctly
- [ ] File upload/download working
- [ ] Email notifications configured (if applicable)

### ✅ User Experience Verification
- [ ] Mobile responsiveness verified
- [ ] Accessibility features working
- [ ] Performance acceptable on various devices
- [ ] Error handling graceful
- [ ] User feedback mechanisms in place

## Post-Deployment

### 1. Immediate Post-Launch Tasks
```bash
# Monitor application logs
tail -f /var/log/physical-ai-textbook/app.log

# Check health endpoints
curl https://your-domain.com/health
curl https://your-domain.com/api/v1/health
```

### 2. Monitoring Setup
- Configure application performance monitoring (APM)
- Set up error tracking
- Configure user analytics
- Set up infrastructure monitoring

### 3. Maintenance Schedule
- Database backups (daily)
- Application logs rotation (daily)
- Security updates (weekly)
- Performance reviews (monthly)

## Rollback Plan

In case of critical issues post-deployment:

1. **Immediate Response**:
   - Switch traffic back to previous version
   - Monitor system status
   - Notify stakeholders

2. **Investigation**:
   - Review logs and error reports
   - Identify root cause
   - Develop fix

3. **Resolution**:
   - Apply fix to staging
   - Test thoroughly
   - Redeploy with fix

## Support and Maintenance

### Contact Information
- **Technical Support**: support@your-organization.com
- **Security Issues**: security@your-organization.com
- **Emergency**: emergency@your-organization.com

### Documentation
- API Documentation: `/docs` and `/redoc`
- User Manual: Available at `/docs/user-documentation.md`
- Admin Guide: `/docs/admin-guide.md`

---

**Deployment Date**: December 18, 2025
**Version**: 1.0.0
**Deployed By**: Physical AI Textbook Team

*This document is maintained as part of the Physical AI & Humanoid Robotics Textbook project.*
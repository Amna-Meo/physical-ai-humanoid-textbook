# Physical AI & Humanoid Robotics Textbook - Backend Deployment Guide

## Table of Contents
1. [Overview](#overview)
2. [Architecture](#architecture)
3. [Prerequisites](#prerequisites)
4. [Environment Setup](#environment-setup)
5. [Containerized Deployment](#containerized-deployment)
6. [Production Deployment](#production-deployment)
7. [Configuration Management](#configuration-management)
8. [Monitoring and Health Checks](#monitoring-and-health-checks)
9. [Scaling and Performance](#scaling-and-performance)
10. [Troubleshooting](#troubleshooting)
11. [Security Best Practices](#security-best-practices)

## Overview

This document provides a comprehensive guide for deploying the Physical AI & Humanoid Robotics Textbook backend service. The backend is built with FastAPI and designed for production deployment using Docker and Docker Compose.

### Key Components
- **Backend API**: FastAPI application serving the core API
- **Database**: PostgreSQL for structured data storage
- **Vector Database**: Qdrant for RAG-based AI responses
- **Reverse Proxy**: Nginx for handling requests and SSL termination

## Architecture

The backend deployment follows a microservices architecture pattern:

```
Internet
    ↓
Nginx Reverse Proxy
    ↓
FastAPI Backend Service
    ↙         ↘
PostgreSQL ←→ Qdrant (Vector DB)
```

### Service Dependencies
- The backend service requires both PostgreSQL and Qdrant to be running
- Nginx acts as a reverse proxy and handles SSL termination
- Health checks ensure all services are operational before accepting traffic

## Prerequisites

### System Requirements
- **Operating System**: Linux (Ubuntu 20.04+), macOS, or Windows with WSL2
- **Docker**: 20.10+ with Docker Compose plugin
- **Git**: 2.30+
- **Storage**: At least 10GB free space for containers and data

### External Services (Production)
- **Domain Name**: For production deployment
- **SSL Certificate**: For HTTPS (recommended: Let's Encrypt)
- **Cloud Provider Account**: AWS, GCP, Azure, or DigitalOcean (optional)

## Environment Setup

### 1. Clone the Repository
```bash
git clone https://github.com/your-organization/physical-ai-textbook.git
cd physical-ai-humanoid-textbook
```

### 2. Prepare Environment Files
Copy the appropriate environment template and configure your values:

```bash
# For production
cp .env.production .env
# Edit .env with your production values
nano .env

# For staging
cp .env.staging .env.staging
# Edit .env.staging with your staging values
nano .env.staging
```

The project includes example environment files:
- `.env.production` - Production environment variables
- `.env.staging` - Staging environment variables
- `.env.example` - Template with documentation

**Important**: Never commit actual API keys or sensitive information to version control.

### 3. Configure Environment Variables

Key environment variables for production:

| Variable | Description | Example |
|----------|-------------|---------|
| `DATABASE_URL` | PostgreSQL connection string | `postgresql://user:pass@db:5432/dbname` |
| `QDRANT_URL` | Qdrant vector database URL | `http://qdrant:6333` |
| `GEMINI_API_KEY` | Google Gemini API key | `your_api_key_here` |
| `OPENAI_API_KEY` | OpenAI API key | `your_api_key_here` |
| `SECRET_KEY` | JWT secret key (32+ chars) | `your_very_long_secret_key_here` |
| `DOMAIN_NAME` | Your domain name | `yourdomain.com` |

## Containerized Deployment

### 1. Build and Deploy Using Docker Compose

```bash
# Deploy for the first time
./deploy_backend.sh deploy

# Update existing deployment
./deploy_backend.sh update

# Check deployment status
./deploy_backend.sh status

# Run health checks
./deploy_backend.sh health
```

### 2. Manual Docker Compose Deployment

```bash
# Build containers
docker-compose build

# Start services in detached mode
docker-compose up -d

# View logs
docker-compose logs -f

# Stop services
docker-compose down
```

### 3. Docker Compose Services

The deployment includes four main services:

#### Backend Service (`backend`)
- Serves the FastAPI application
- Handles API requests on port 8000
- Runs database migrations on startup
- Connects to PostgreSQL and Qdrant

#### Database Service (`db`)
- PostgreSQL 15 container
- Persistent volume for data storage
- Health check for readiness

#### Vector Database Service (`qdrant`)
- Qdrant vector database
- Persistent volume for embeddings
- Health check for readiness

#### Reverse Proxy Service (`nginx`)
- Nginx server for routing
- SSL termination
- Static file serving

## Production Deployment

### 1. Pre-Deployment Checklist

Before deploying to production:

- [ ] All tests pass locally
- [ ] Environment variables are properly configured
- [ ] SSL certificates are in place
- [ ] Domain DNS is pointing to the server
- [ ] Firewall rules are configured
- [ ] Backup strategy is in place

### 2. Production Deployment Steps

```bash
# 1. Set production environment
export ENVIRONMENT=production

# 2. Run pre-deployment tests
./deploy_backend.sh test

# 3. Deploy to production
./deploy_backend.sh deploy

# 4. Verify deployment
./deploy_backend.sh health
./deploy_backend.sh status
```

### 3. Post-Deployment Tasks

After successful deployment:

1. **Verify API Access**
   ```bash
   curl http://yourdomain.com/health
   curl http://yourdomain.com/api/v1/docs
   ```

2. **Test Database Connectivity**
   ```bash
   curl http://yourdomain.com/api/v1/health
   ```

3. **Verify AI Services**
   - Test AI chat endpoints
   - Verify RAG functionality

4. **Monitor Resource Usage**
   - CPU and memory utilization
   - Database connections
   - API response times

## Configuration Management

### 1. Environment Files

Different environment files for various stages:

- `.env.production` - Production environment
- `.env.staging` - Staging environment
- `.env.development` - Development environment

### 2. Configuration Best Practices

1. **Never commit secrets** to version control
2. **Use strong passwords** for database users
3. **Rotate API keys** regularly
4. **Enable SSL/TLS** for all production traffic
5. **Use non-root users** in containers

### 3. Secrets Management

For production deployments, consider using:

- HashiCorp Vault
- AWS Secrets Manager
- Azure Key Vault
- Kubernetes secrets (if using K8s)

## Monitoring and Health Checks

### 1. Built-in Health Endpoints

The application provides several health check endpoints:

- `GET /health` - Basic health check
- `GET /api/v1/health` - API health check
- `GET /docs` - API documentation
- `GET /redoc` - Alternative API documentation

### 2. Docker Compose Health Checks

Each service has built-in health checks:

- **Backend**: HTTP request to `/health`
- **Database**: PostgreSQL readiness check
- **Qdrant**: HTTP request to `/ready`

### 3. Monitoring Commands

```bash
# View all container logs
docker-compose logs -f

# Monitor specific service
docker-compose logs -f backend

# Check resource usage
docker stats

# Monitor backend logs with filtering
docker-compose logs -f backend | grep -E "(ERROR|WARNING)"
```

### 4. Automated Health Monitoring

Set up automated monitoring with:

- Prometheus + Grafana
- ELK Stack (Elasticsearch, Logstash, Kibana)
- CloudWatch (AWS) or similar cloud monitoring

## Scaling and Performance

### 1. Horizontal Scaling

To scale the backend service:

```bash
# Scale backend to 3 instances
docker-compose up -d --scale backend=3

# Check running instances
docker-compose ps
```

### 2. Performance Tuning

Adjust the following parameters for better performance:

- `WORKERS`: Number of backend workers (typically CPU cores)
- `MAX_REQUESTS`: Restart workers after processing requests
- `TIMEOUT`: Request timeout value
- `KEEPALIVE`: Connection keep-alive duration

### 3. Database Optimization

For production databases:

- Enable connection pooling
- Configure appropriate memory limits
- Set up read replicas if needed
- Regular maintenance tasks

## Troubleshooting

### 1. Common Issues

#### Container Won't Start
```bash
# Check container logs
docker-compose logs backend

# Check if dependencies are ready
docker-compose logs db
docker-compose logs qdrant
```

#### Database Connection Issues
```bash
# Verify database is running
docker-compose exec db pg_isready

# Check database logs
docker-compose logs db
```

#### API Not Responding
```bash
# Check backend logs
docker-compose logs backend

# Test health endpoint inside container
docker-compose exec backend curl localhost:8000/health
```

### 2. Debugging Commands

```bash
# Enter backend container
docker-compose exec backend bash

# Check environment variables
docker-compose exec backend env | grep -E "(DATABASE|QDRANT|API)"

# Test database connection
docker-compose exec backend python -c "
import psycopg2
conn = psycopg2.connect('${DATABASE_URL}')
print('Database connection successful')
conn.close()
"

# Test Qdrant connection
docker-compose exec backend python -c "
import qdrant_client
client = qdrant_client.QdrantClient(url='${QDRANT_URL}', api_key='${QDRANT_API_KEY}')
print('Qdrant connection successful')
"
```

### 3. Recovery Procedures

#### Rollback to Previous Version
```bash
./deploy_backend.sh rollback
```

#### Force Rebuild
```bash
# Rebuild and restart all services
docker-compose down
docker-compose build --no-cache
docker-compose up -d
```

#### Database Recovery
```bash
# Restore from backup
docker cp backup.sql textbook-db:/tmp/
docker-compose exec db psql -U textbook_user -d physical_ai_textbook -f /tmp/backup.sql
```

## Security Best Practices

### 1. Network Security

- Use private networks for internal services
- Limit exposed ports
- Implement firewall rules
- Use SSL/TLS for all traffic

### 2. Secret Management

- Store secrets in environment variables (not in images)
- Rotate API keys regularly
- Use least privilege principle
- Audit access to sensitive data

### 3. Container Security

- Use minimal base images (Alpine Linux)
- Run containers as non-root users
- Scan images for vulnerabilities
- Keep images updated

### 4. API Security

- Implement rate limiting
- Validate all inputs
- Use JWT for authentication
- Enable CORS appropriately

---

## Support and Maintenance

### Daily Tasks
- Monitor application logs
- Check disk space usage
- Verify backup jobs

### Weekly Tasks
- Update container images
- Review security alerts
- Test backup restoration

### Monthly Tasks
- Rotate API keys
- Review access logs
- Performance optimization

### Contact Information
- **Technical Support**: support@your-organization.com
- **Security Issues**: security@your-organization.com

---

**Document Version**: 1.0  
**Last Updated**: December 22, 2025  
**Next Review**: January 22, 2026
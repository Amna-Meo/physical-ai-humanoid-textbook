# Deploying Physical AI & Humanoid Robotics Textbook Backend (Without Docker)

## Overview
This guide explains how to deploy the backend services without Docker, which is useful when Docker is not available on your system. The frontend is already deployed on Vercel.

## Prerequisites
- Python 3.11+ installed
- pip package manager
- Virtual environment tools (venv)
- System dependencies as per requirements.txt
- Access to PostgreSQL database (for production)
- Qdrant vector database access
- API keys for AI services (OpenAI, Google Gemini)

## Step 1: Prepare Environment

### Create a Virtual Environment
```bash
python3 -m venv backend_env
source backend_env/bin/activate  # On Windows: backend_env\Scripts\activate
```

### Install Dependencies
```bash
# Activate your virtual environment first
pip install --upgrade pip
pip install -r requirements.txt
```

## Step 2: Configure Environment Variables

Copy and configure your environment file:
```bash
cp .env.production .env
# Edit .env with your production values
nano .env
```

Key environment variables:
- `DATABASE_URL`: PostgreSQL connection string
- `QDRANT_URL`: Qdrant vector database URL
- `GEMINI_API_KEY`: Google Gemini API key
- `OPENAI_API_KEY`: OpenAI API key
- `SECRET_KEY`: JWT secret key (32+ chars)
- `ALLOWED_ORIGINS`: Comma-separated list of allowed origins (include your Vercel frontend URL)

## Step 3: Database Setup

### For Production Database
```bash
# Run database migrations
cd backend
python -m alembic upgrade head
```

### For Development/Testing
The application can use SQLite as well, which requires no additional setup.

## Step 4: Run the Backend Server

### Using the Production Script
```bash
cd backend
python start_server.py
```

### Using Uvicorn Directly
```bash
cd backend
uvicorn src.api.main:app --host 0.0.0.0 --port 8000 --workers 4
```

## Step 5: Configure Nginx (Optional but Recommended)

If you have nginx installed, use the configuration in `nginx.production.conf` to:

1. Proxy API requests to your backend server
2. Handle SSL termination
3. Redirect non-API traffic to your Vercel frontend

### Update nginx configuration:
1. Replace `your-domain.com` with your actual domain
2. Replace `https://your-vercel-project.vercel.app` with your actual Vercel URL
3. Update backend server address if needed (default: localhost:8000)
4. Install SSL certificates if using HTTPS

## Step 6: Production Considerations

### Use a Process Manager
For production, use a process manager like systemd, pm2, or supervisord:

#### Example systemd service file (place in /etc/systemd/system/textbook-backend.service):
```
[Unit]
Description=Physical AI Textbook Backend
After=network.target

[Service]
Type=simple
User=your-user
WorkingDirectory=/path/to/physical-ai-humanoid-textbook/backend
EnvironmentFile=/path/to/physical-ai-humanoid-textbook/.env
ExecStart=/path/to/backend_env/bin/python start_server.py
Restart=always

[Install]
WantedBy=multi-user.target
```

Then:
```bash
sudo systemctl daemon-reload
sudo systemctl enable textbook-backend
sudo systemctl start textbook-backend
```

### Security Considerations
- Use HTTPS in production
- Restrict `ALLOWED_ORIGINS` to only your frontend domains
- Use strong, rotated API keys
- Implement proper firewall rules
- Regular security updates

### Monitoring
- Set up logging to external service
- Monitor application performance
- Set up health checks
- Configure alerting for failures

## Step 7: Connecting Frontend to Backend

Since your frontend is deployed on Vercel:
1. Update your frontend's API configuration to point to your backend server
2. Ensure CORS settings in your backend allow requests from your Vercel domain
3. Test the integration

## Troubleshooting

### Common Issues:
1. **Database Connection**: Ensure PostgreSQL is running and accessible
2. **Port Conflicts**: Check if port 8000 (or your chosen port) is available
3. **Environment Variables**: Verify all required environment variables are set
4. **API Keys**: Confirm AI service keys are valid and have proper permissions

### Health Check
Visit `http://your-server:8000/health` to verify the backend is running.

## Next Steps

1. Test the API endpoints to ensure they work with your Vercel frontend
2. Monitor application logs for any issues
3. Set up automated backups for your database
4. Implement proper monitoring and alerting
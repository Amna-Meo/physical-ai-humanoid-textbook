#!/bin/bash
# Deployment script for Hugging Face Spaces

set -e  # Exit on any error

echo "Preparing repository for Hugging Face Spaces deployment..."

# Create a .spaceignore file to exclude unnecessary files
cat > .spaceignore << 'EOF'
.git
.gitignore
__pycache__
*.pyc
*.pyo
*.pyd
.Python
env/
venv/
.venv/
pip-log.txt
pip-delete-this-directory.txt
.tox
.coverage
.coverage.*
.cache
nosetests.xml
coverage.xml
*.cover
*.log
.git/
.mypy_cache
.pytest_cache
.hypothesis
.DS_Store
README.md
BACKEND_DEPLOYMENT.md
BACKEND_DEPLOYMENT_NO_DOCKER.md
DEPLOYMENT_GUIDE.md
CONTRIBUTING.md
IMPLEMENTATION_SUMMARY.md
INTEGRATION_TEST_PLAN.md
PROJECT_OVERVIEW.md
CLAUDE.md
ENVIRONMENT_VARIABLES.md
create_chapter_metadata.py
demo_multi_provider.py
deploy.sh
deploy_backend.sh
docker-compose.yml
nginx.conf
nginx.production.conf
ssl/
frontend/
backend/migrations/
backend/physical-ai-humanoid-textbook/
history/
research/
spec/
specs/
.spacex/
EOF

echo ".spaceignore file created"

# Create/update the main app.py file if needed
if [ ! -f app.py ]; then
    echo "app.py not found. Please ensure it exists."
    exit 1
fi

echo "Repository prepared for Hugging Face Spaces deployment!"
echo ""
echo "To deploy to Hugging Face Spaces:"
echo "1. Go to https://huggingface.co/spaces"
echo "2. Click 'Create Space'"
echo "3. Choose 'Docker' as SDK"
echo "4. Connect to your Git repository"
echo "5. Use the following Dockerfile: Dockerfile.hf"
echo "6. The app will run on port 7860"
echo ""
echo "Alternatively, you can use the Hugging Face CLI:"
echo "1. Install: pip install huggingface_hub[cli]"
echo "2. Login: huggingface-cli login"
echo "3. Create space: hf space create --repo-id your-username/your-space-name --sdk docker ."
echo ""
echo "Make sure to add any required environment variables in the Space settings if needed."
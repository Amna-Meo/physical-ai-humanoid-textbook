#!/bin/bash

# Deployment script for Physical AI & Humanoid Robotics Textbook
# This script handles deployment to GitHub Pages and Vercel

set -e  # Exit on any error

# Configuration
REPO_NAME="physical-ai-textbook"
GITHUB_PAGES_BRANCH="gh-pages"
VERCEL_PROJECT_NAME="physical-ai-textbook"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}Starting deployment process for Physical AI Textbook${NC}"
echo "=================================================="

# Function to print status
print_status() {
    echo -e "${YELLOW}[$1]${NC} $2"
}

# Function to print success
print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $2"
}

# Function to print error
print_error() {
    echo -e "${RED}[ERROR]${NC} $2"
    exit 1
}

# Check if required tools are installed
check_prerequisites() {
    print_status "CHECK" "Verifying prerequisites..."

    if ! command -v git &> /dev/null; then
        print_error "Git is not installed"
    fi

    if ! command -v npm &> /dev/null; then
        print_error "npm is not installed"
    fi

    if ! command -v python3 &> /dev/null; then
        print_error "Python 3 is not installed"
    fi

    # Check if we're in the correct directory
    if [ ! -f "pyproject.toml" ] || [ ! -d "backend" ] || [ ! -d "frontend" ]; then
        print_error "Not in the project root directory"
    fi

    print_success "Prerequisites verified"
}

# Build the frontend
build_frontend() {
    print_status "BUILD" "Building frontend application..."

    if [ ! -d "frontend" ]; then
        print_error "Frontend directory not found"
    fi

    cd frontend

    # Install dependencies
    print_status "INSTALL" "Installing frontend dependencies..."
    npm ci --no-audit --no-fund

    # Build for production
    print_status "BUILD" "Creating production build..."
    npm run build

    if [ $? -ne 0 ]; then
        print_error "Frontend build failed"
    fi

    print_success "Frontend built successfully"
    cd ..
}

# Build the backend
build_backend() {
    print_status "BUILD" "Building backend application..."

    if [ ! -d "backend" ]; then
        print_error "Backend directory not found"
    fi

    # Install Python dependencies
    print_status "INSTALL" "Installing backend dependencies..."

    # Check if we're in a virtual environment
    if [ -z "$VIRTUAL_ENV" ]; then
        print_status "VENV" "Creating virtual environment..."
        python3 -m venv venv
        source venv/bin/activate
    fi

    pip install --upgrade pip
    pip install -r requirements.txt

    print_success "Backend dependencies installed"
}

# Deploy to GitHub Pages
deploy_to_github_pages() {
    print_status "DEPLOY" "Deploying to GitHub Pages..."

    # Check if we're in a git repository
    if ! git rev-parse --git-dir > /dev/null 2>&1; then
        print_error "Not in a git repository"
    fi

    # Save current branch
    CURRENT_BRANCH=$(git branch --show-current)

    # Create or switch to gh-pages branch
    git checkout -B $GITHUB_PAGES_BRANCH

    # Remove all files except .git
    git rm -rf . 2>/dev/null || true

    # Copy built frontend to root (for static hosting)
    if [ -d "frontend/build" ]; then
        cp -r frontend/build/* .
    else
        print_error "Frontend build directory not found"
    fi

    # Create a simple index.html if one doesn't exist
    if [ ! -f "index.html" ]; then
        cat > index.html << EOF
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Physical AI & Humanoid Robotics Textbook</title>
    <style>
        body { font-family: Arial, sans-serif; text-align: center; padding: 50px; }
        .container { max-width: 800px; margin: 0 auto; }
    </style>
</head>
<body>
    <div class="container">
        <h1>Physical AI & Humanoid Robotics Textbook</h1>
        <p>This site is under construction. The full application will be available soon.</p>
        <p>For the backend API, please see the deployed service.</p>
    </div>
</body>
</html>
EOF
    fi

    # Add CNAME file if needed (uncomment and customize if you have a custom domain)
    # echo "yourdomain.com" > CNAME

    # Add all files
    git add .

    # Commit changes
    git config user.name "github-actions[bot]"
    git config user.email "github-actions[bot]@users.noreply.github.com"
    git commit -m "Deploy to GitHub Pages - $(date)"

    # Push to GitHub Pages
    git push origin $GITHUB_PAGES_BRANCH

    # Return to original branch
    git checkout $CURRENT_BRANCH

    print_success "Deployed to GitHub Pages"
}

# Create Vercel deployment configuration
create_vercel_config() {
    print_status "CONFIG" "Creating Vercel configuration..."

    # Create vercel.json if it doesn't exist
    if [ ! -f "vercel.json" ]; then
        cat > vercel.json << EOF
{
  "version": 2,
  "name": "$REPO_NAME",
  "builds": [
    {
      "src": "frontend/**",
      "use": "@vercel/static"
    },
    {
      "src": "backend/src/**",
      "use": "@vercel/python",
      "config": { "runtime": "python3.11" }
    }
  ],
  "routes": [
    {
      "src": "/api/(.*)",
      "dest": "/backend/src/api/\$1"
    },
    {
      "src": "/(.*)",
      "dest": "/frontend/build/\$1",
      "continue": true
    }
  ],
  "env": {
    "DATABASE_URL": "\$DATABASE_URL",
    "OPENAI_API_KEY": "\$OPENAI_API_KEY"
  }
}
EOF
        print_success "Vercel configuration created"
    else
        print_status "SKIP" "Vercel configuration already exists"
    fi
}

# Create GitHub Actions workflow for automated deployment
create_github_workflow() {
    print_status "CONFIG" "Creating GitHub Actions workflow..."

    # Create .github/workflows/deploy.yml
    mkdir -p .github/workflows

    cat > .github/workflows/deploy.yml << EOF
name: Deploy to Production

on:
  push:
    branches: [ main, master ]
  pull_request:
    branches: [ main, master ]

jobs:
  deploy:
    runs-on: ubuntu-latest

    steps:
    - name: Checkout code
      uses: actions/checkout@v3

    - name: Setup Node.js
      uses: actions/setup-node@v3
      with:
        node-version: '18'
        cache: 'npm'
        cache-dependency-path: 'frontend/package-lock.json'

    - name: Setup Python
      uses: actions/setup-python@v4
      with:
        python-version: '3.11'
        cache: 'pip'

    - name: Install backend dependencies
      run: |
        python -m pip install --upgrade pip
        pip install -r requirements.txt
      working-directory: .

    - name: Install frontend dependencies
      run: npm ci --no-audit --no-fund
      working-directory: frontend

    - name: Build frontend
      run: npm run build
      working-directory: frontend
      env:
        CI: false

    - name: Run backend tests
      run: python -m pytest backend/tests/
      if: github.event_name == 'pull_request'

    - name: Deploy to Vercel
      if: github.ref == 'refs/heads/main' && github.event_name == 'push'
      run: |
        npm install -g vercel@latest
        vercel --prod --token=\$VERCEL_TOKEN
      env:
        VERCEL_TOKEN: \${{ secrets.VERCEL_TOKEN }}
        VERCEL_PROJECT_ID: \${{ secrets.VERCEL_PROJECT_ID }}
        VERCEL_ORG_ID: \${{ secrets.VERCEL_ORG_ID }}

    - name: Deploy to GitHub Pages
      if: github.ref == 'refs/heads/main' && github.event_name == 'push'
      uses: peaceiris/actions-gh-pages@v3
      with:
        github_token: \${{ secrets.GITHUB_TOKEN }}
        publish_dir: ./frontend/build
        publish_branch: gh-pages
EOF

    print_success "GitHub Actions workflow created"
}

# Run end-to-end tests before deployment
run_e2e_tests() {
    print_status "TEST" "Running end-to-end tests..."

    # Check if test files exist
    if [ -f "test_student_learning_journey_complete.py" ]; then
        python test_student_learning_journey_complete.py
        if [ $? -ne 0 ]; then
            print_error "End-to-end tests failed"
        fi
        print_success "End-to-end tests passed"
    else
        print_status "SKIP" "No end-to-end tests found to run"
    fi
}

# Main deployment function
main() {
    print_status "START" "Starting deployment process..."

    # Run checks and builds
    check_prerequisites
    build_backend
    build_frontend
    run_e2e_tests

    # Deployment options
    if [ "$1" = "github-pages" ]; then
        deploy_to_github_pages
    elif [ "$1" = "vercel" ]; then
        create_vercel_config
        print_status "INFO" "Vercel configuration created. Deploy using 'vercel --prod' command."
    elif [ "$1" = "all" ]; then
        create_vercel_config
        create_github_workflow
        deploy_to_github_pages
        print_status "INFO" "Full deployment configuration created. Deploy to Vercel using 'vercel --prod' command."
    else
        create_vercel_config
        create_github_workflow
        print_status "INFO" "Deployment configuration created. Use './deploy.sh github-pages' or './deploy.sh vercel' for specific deployments."
    fi

    print_success "Deployment process completed!"
    echo
    echo -e "${GREEN}Next steps:${NC}"
    echo "1. For Vercel: Install Vercel CLI with 'npm install -g vercel@latest'"
    echo "2. Link your project: 'vercel'"
    echo "3. Deploy: 'vercel --prod'"
    echo "4. For GitHub Pages: The site is already deployed"
}

# Display help
show_help() {
    echo "Physical AI Textbook Deployment Script"
    echo
    echo "Usage: $0 [option]"
    echo
    echo "Options:"
    echo "  all           Deploy to all platforms (default)"
    echo "  github-pages  Deploy to GitHub Pages"
    echo "  vercel        Prepare Vercel configuration"
    echo "  help          Show this help message"
    echo
    echo "Examples:"
    echo "  $0                    # Create all configurations"
    echo "  $0 github-pages       # Deploy to GitHub Pages"
    echo "  $0 vercel             # Prepare Vercel configuration"
}

# Parse command line arguments
case "${1:-all}" in
    "all")
        main all
        ;;
    "github-pages")
        main github-pages
        ;;
    "vercel")
        main vercel
        ;;
    "help"|"-h"|"--help")
        show_help
        ;;
    *)
        echo -e "${RED}Unknown option: $1${NC}"
        show_help
        exit 1
        ;;
esac
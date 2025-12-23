#!/bin/bash

# Production Deployment Script for Physical AI & Humanoid Robotics Textbook Backend
# This script handles production deployment with multiple options

set -e  # Exit on any error

# Configuration
APP_NAME="physical-ai-textbook-backend"
ENVIRONMENT=${ENVIRONMENT:-production}
COMPOSE_FILE="docker-compose.yml"
BACKEND_DIR="./backend"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${GREEN}Starting ${APP_NAME} deployment for ${ENVIRONMENT} environment${NC}"
echo "================================================================"

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

# Function to print info
print_info() {
    echo -e "${BLUE}[INFO]${NC} $2"
}

# Check if required tools are installed
check_prerequisites() {
    print_status "CHECK" "Verifying prerequisites..."

    if ! command -v docker &> /dev/null; then
        print_error "Docker is not installed. Please install Docker first."
    fi

    if ! command -v docker-compose &> /dev/null; then
        print_error "Docker Compose is not installed. Please install Docker Compose."
    fi

    if [ ! -f ".env.${ENVIRONMENT}" ]; then
        print_error ".env.${ENVIRONMENT} file not found. Please create the environment file."
    fi

    if [ ! -d "${BACKEND_DIR}" ]; then
        print_error "Backend directory not found at ${BACKEND_DIR}"
    fi

    print_success "Prerequisites verified"
}

# Load environment variables
load_environment() {
    print_status "LOAD" "Loading ${ENVIRONMENT} environment variables..."

    if [ -f ".env.${ENVIRONMENT}" ]; then
        # Source the environment file, ignoring comments and empty lines
        set -a
        source <(grep -v '^#' .env.${ENVIRONMENT} | grep -v '^$')
        set +a
        print_success "Environment variables loaded from .env.${ENVIRONMENT}"
    else
        print_error ".env.${ENVIRONMENT} file not found"
    fi
}

# Build the application
build_application() {
    print_status "BUILD" "Building application containers..."

    # Copy the appropriate environment file, excluding comments
    grep -v '^#' .env.${ENVIRONMENT} | grep -v '^$' > .env

    # Build the containers
    docker-compose -f ${COMPOSE_FILE} build --no-cache

    print_success "Application containers built successfully"
}

# Deploy to production
deploy_production() {
    print_status "DEPLOY" "Deploying to production..."

    # Stop existing containers if running
    docker-compose -f ${COMPOSE_FILE} down --remove-orphans || true

    # Start the services
    docker-compose -f ${COMPOSE_FILE} up -d

    # Wait for services to be healthy
    print_status "WAIT" "Waiting for services to be healthy..."
    
    # Wait for database
    timeout 120 sh -c 'until docker-compose -f '${COMPOSE_FILE}' exec db pg_isready > /dev/null 2>&1; do sleep 5; done' || {
        print_error "Database failed to become ready"
    }

    # Wait for Qdrant
    timeout 60 sh -c 'until curl -s http://localhost:6333/ready > /dev/null 2>&1; do sleep 5; done' || {
        print_error "Qdrant failed to become ready"
    }

    # Wait for backend
    timeout 120 sh -c 'until curl -s http://localhost:8000/health > /dev/null 2>&1; do sleep 5; done' || {
        print_error "Backend failed to become ready"
    }

    print_success "Application deployed and healthy"
}

# Run database migrations
run_migrations() {
    print_status "MIGRATE" "Running database migrations..."

    # Execute migrations in the backend container
    docker-compose -f ${COMPOSE_FILE} exec backend alembic upgrade head

    print_success "Database migrations completed"
}

# Run tests before deployment
run_pre_deployment_tests() {
    print_status "TEST" "Running pre-deployment tests..."

    # Run backend tests in the container
    docker-compose -f ${COMPOSE_FILE} exec backend python -m pytest tests/ -v

    print_success "Pre-deployment tests passed"
}

# Backup current deployment
backup_current() {
    print_status "BACKUP" "Creating backup of current deployment..."

    TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
    BACKUP_DIR="backups/${TIMESTAMP}"
    mkdir -p ${BACKUP_DIR}

    # Backup database if running
    if docker-compose -f ${COMPOSE_FILE} ps db > /dev/null 2>&1; then
        docker-compose -f ${COMPOSE_FILE} exec db pg_dump -U textbook_user -d physical_ai_textbook > ${BACKUP_DIR}/db_backup_${TIMESTAMP}.sql
        print_info "Database backed up to ${BACKUP_DIR}/db_backup_${TIMESTAMP}.sql"
    fi

    # Backup configuration
    cp .env.${ENVIRONMENT} ${BACKUP_DIR}/
    cp docker-compose.yml ${BACKUP_DIR}/
    print_info "Configuration backed up to ${BACKUP_DIR}/"

    print_success "Backup completed to ${BACKUP_DIR}"
}

# Rollback to previous version
rollback() {
    print_status "ROLLBACK" "Rolling back to previous version..."

    if [ -d "backups" ]; then
        LATEST_BACKUP=$(ls -td backups/*/ | head -n 1)
        if [ -n "$LATEST_BACKUP" ]; then
            print_info "Restoring from backup: $LATEST_BACKUP"
            
            # Stop current services
            docker-compose -f ${COMPOSE_FILE} down
            
            # Restore database if backup exists
            DB_BACKUP="${LATEST_BACKUP}/db_backup_*.sql"
            if ls $DB_BACKUP 1> /dev/null 2>&1; then
                RESTORE_FILE=$(ls $DB_BACKUP | head -n 1)
                docker-compose -f ${COMPOSE_FILE} exec -T db psql -U textbook_user -d physical_ai_textbook < $RESTORE_FILE
                print_info "Database restored from $RESTORE_FILE"
            fi
            
            print_success "Rollback completed"
        else
            print_error "No backups found for rollback"
        fi
    else
        print_error "No backups directory found"
    fi
}

# Health check
health_check() {
    print_status "HEALTH" "Performing health check..."

    # Check backend health
    BACKEND_HEALTH=$(curl -s -o /dev/null -w "%{http_code}" http://localhost:8000/health)
    if [ "$BACKEND_HEALTH" -eq 200 ]; then
        print_info "Backend health: OK ($BACKEND_HEALTH)"
    else
        print_error "Backend health check failed: $BACKEND_HEALTH"
    fi

    # Check if all services are running
    RUNNING_SERVICES=$(docker-compose -f ${COMPOSE_FILE} ps --status running --format "{{.Service}}")
    EXPECTED_SERVICES=("db" "qdrant" "backend" "nginx")
    
    for service in "${EXPECTED_SERVICES[@]}"; do
        if [[ $RUNNING_SERVICES =~ (^|[[:space:]])$service($|[[:space:]]) ]]; then
            print_info "Service $service: RUNNING"
        else
            print_error "Service $service: NOT RUNNING"
        fi
    done

    print_success "Health check completed"
}

# Clean up unused resources
cleanup() {
    print_status "CLEAN" "Cleaning up unused resources..."

    # Remove unused containers, networks, images, and build cache
    docker system prune -f --volumes
    docker-compose -f ${COMPOSE_FILE} down -v

    print_success "Cleanup completed"
}

# Show deployment status
show_status() {
    print_status "STATUS" "Showing deployment status..."

    docker-compose -f ${COMPOSE_FILE} ps

    echo
    print_info "Application endpoints:"
    print_info "  - Backend API: http://localhost:8000"
    print_info "  - Backend Docs: http://localhost:8000/docs"
    print_info "  - Qdrant: http://localhost:6333"
    print_info "  - Health Check: http://localhost:8000/health"
}

# Main deployment function
main() {
    ACTION=${1:-"deploy"}

    case $ACTION in
        "deploy")
            print_status "START" "Starting deployment process..."
            check_prerequisites
            load_environment
            backup_current
            build_application
            deploy_production
            run_migrations
            health_check
            show_status
            ;;
        "update")
            print_status "UPDATE" "Updating existing deployment..."
            check_prerequisites
            load_environment
            backup_current
            build_application
            docker-compose -f ${COMPOSE_FILE} up -d --no-deps --build backend
            run_migrations
            health_check
            ;;
        "rollback")
            rollback
            ;;
        "health")
            check_prerequisites
            load_environment
            health_check
            ;;
        "status")
            check_prerequisites
            show_status
            ;;
        "cleanup")
            cleanup
            ;;
        "test")
            check_prerequisites
            load_environment
            run_pre_deployment_tests
            ;;
        *)
            print_error "Unknown action: $ACTION"
            show_help
            exit 1
            ;;
    esac
}

# Display help
show_help() {
    echo "Physical AI Textbook Backend Deployment Script"
    echo
    echo "Usage: $0 [action]"
    echo
    echo "Actions:"
    echo "  deploy      Deploy the application (default)"
    echo "  update      Update the application with new changes"
    echo "  rollback    Rollback to previous version"
    echo "  health      Perform health check"
    echo "  status      Show deployment status"
    echo "  cleanup     Clean up unused resources"
    echo "  test        Run pre-deployment tests"
    echo "  help        Show this help message"
    echo
    echo "Environment:"
    echo "  ENVIRONMENT  Set to 'production', 'staging', or 'development' (default: production)"
    echo
    echo "Examples:"
    echo "  $0 deploy                    # Deploy to production"
    echo "  ENVIRONMENT=staging $0 deploy # Deploy to staging"
    echo "  $0 health                    # Check health status"
}

# Parse command line arguments
case "${1:-deploy}" in
    "deploy"|"update"|"rollback"|"health"|"status"|"cleanup"|"test")
        main $1
        ;;
    "help"|"-h"|"--help")
        show_help
        ;;
    "")
        main "deploy"
        ;;
    *)
        echo -e "${RED}Unknown option: $1${NC}"
        show_help
        exit 1
        ;;
esac

print_success "Deployment process completed!"
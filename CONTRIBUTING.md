# Contributing to Physical AI & Humanoid Robotics Textbook

We welcome contributions to the Physical AI & Humanoid Robotics Textbook project! This document outlines the guidelines for contributing to this project.

## Table of Contents
- [Getting Started](#getting-started)
- [Development Workflow](#development-workflow)
- [Code Style](#code-style)
- [Documentation](#documentation)
- [Testing](#testing)
- [Submitting Changes](#submitting-changes)

## Getting Started

1. Fork the repository
2. Clone your fork: `git clone https://github.com/your-username/physical-ai-textbook.git`
3. Create a virtual environment: `python -m venv venv`
4. Activate the virtual environment: `source venv/bin/activate`
5. Install dependencies: `pip install -r requirements.txt`
6. Install frontend dependencies: `cd frontend && npm install`

## Development Workflow

1. Create a new branch for your feature/bug fix: `git checkout -b feature/your-feature-name`
2. Make your changes
3. Write tests if applicable
4. Run tests: `pytest tests/`
5. Commit your changes using conventional commits
6. Push to your fork: `git push origin feature/your-feature-name`
7. Create a pull request

## Code Style

### Python
- Follow PEP 8 style guide
- Use Black for code formatting
- Use type hints where possible
- Write docstrings for all public functions and classes

### JavaScript/React
- Use Prettier for code formatting
- Follow Airbnb JavaScript style guide
- Use functional components with hooks when possible
- Write JSDoc for complex functions

## Documentation

- Update documentation when adding new features
- Write clear, concise comments
- Follow APA (7th Edition) citation format for academic content
- Include examples where appropriate

## Testing

- Write unit tests for all new functionality
- Ensure all tests pass before submitting a pull request
- Aim for high test coverage on critical paths

## Submitting Changes

1. Ensure all tests pass
2. Update documentation as needed
3. Create a descriptive pull request
4. Link to any relevant issues
5. Request review from maintainers
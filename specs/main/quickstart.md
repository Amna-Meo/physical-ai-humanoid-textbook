# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

## Getting Started

This guide will help you set up and start using the Physical AI & Humanoid Robotics textbook platform.

## Prerequisites

- Basic Python programming skills
- Fundamental understanding of AI concepts
- Access to the textbook platform (web-based, no local installation required)

## Technology Stack

The textbook platform is built using:

- **Frontend**: Docusaurus for content delivery
- **Backend**: FastAPI for agent interactions
- **Vector Database**: Qdrant for RAG-based retrieval
- **Storage**: Neon Postgres for structured data
- **AI Integration**: Claude Code for content generation and assistance

## Development Environment Setup

1. **Clone the repository** (for contributors):
   ```bash
   git clone [repository-url]
   cd physical-ai-textbook
   ```

2. **Install dependencies** (for contributors):
   ```bash
   # Backend dependencies
   pip install fastapi uvicorn python-multipart

   # Vector database client
   pip install qdrant-client

   # Database client
   pip install asyncpg
   ```

3. **Environment variables**:
   Create a `.env` file with:
   ```
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_api_key
   DATABASE_URL=your_neon_db_url
   CLAUDE_API_KEY=your_claude_api_key
   ```

## Running the Platform

### For Local Development (Contributors)

1. **Start the backend**:
   ```bash
   uvicorn main:app --reload
   ```

2. **Start the frontend** (if running locally):
   ```bash
   cd frontend
   npm install
   npm start
   ```

### For Users (Standard Access)

1. Access the platform at: [platform-url]
2. Create an account or log in
3. Complete the technical background assessment
4. Begin exploring chapters or use the AI assistant

## Chapter Development Process

If you're contributing content, follow this research-concurrent approach:

1. **Research Phase**: Gather 15-25 authoritative sources for your chapter
2. **Foundation Phase**: Create the basic structure and core concepts
3. **Analysis Phase**: Examine relationships and conflicting findings
4. **Synthesis Phase**: Integrate all elements into coherent content

## Using the AI Assistant

The AI assistant is grounded in textbook content and will:
- Answer questions based only on indexed textbook content
- Provide citations to specific textbook sections
- Maintain ≥95% accuracy in responses
- Never introduce uncited claims

## Content Standards

All content must meet these standards:
- 3,000-6,000 words per chapter
- 15-25 authoritative sources per chapter
- APA (7th Edition) citation format
- Clear learning objectives
- Machine-readable formatting for AI processing

## Quality Validation

All content undergoes validation for:
- Technical accuracy against ROS 2, Gazebo, and NVIDIA Isaac standards
- Pedagogical effectiveness
- AI response relevance (>95%)
- Readability without AI assistance
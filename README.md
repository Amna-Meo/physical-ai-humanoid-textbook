# Physical AI & Humanoid Robotics Textbook

An AI-native textbook platform for Physical AI and Humanoid Robotics education, combining authoritative academic content with interactive AI-powered learning experiences.

## Overview

This project is an AI-native textbook focused on Physical AI and Humanoid Robotics. It combines rigorous academic content with interactive AI-powered features to enhance learning outcomes for students, instructors, and professionals in the field.

### Features

- **AI-Powered Learning**: Interactive AI chat to explain complex concepts grounded in textbook content
- **Structured Content**: Well-researched chapters with 15-25 authoritative sources each
- **Personalized Learning Paths**: Adaptive content delivery based on user background and progress
- **Simulation Integration**: Examples and exercises using ROS 2, Gazebo, and NVIDIA Isaac Sim
- **Multi-Language Support**: Content available in multiple languages including Urdu translation

## Technology Stack

- **Backend**: FastAPI (Python 3.11)
- **Frontend**: Docusaurus (React-based)
- **Database**: Neon Postgres (structured data)
- **Vector Store**: Qdrant (for RAG-based AI responses)
- **AI Integration**: OpenAI API for content understanding

## Getting Started

### Prerequisites

- Python 3.11+
- Node.js 18+ / npm
- Git

### Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/your-username/physical-ai-textbook.git
   cd physical-ai-textbook
   ```

2. Set up the Python virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   pip install -r requirements.txt
   ```

3. Set up the frontend:
   ```bash
   cd frontend
   npm install
   ```

4. Set up environment variables:
   ```bash
   cp .env.example .env
   # Edit .env with your configuration
   ```

### Running the Application

1. **Backend**:
   ```bash
   cd backend
   source ../venv/bin/activate
   uvicorn src.api.main:app --reload
   ```

2. **Frontend**:
   ```bash
   cd frontend
   npm start
   ```

## Project Structure

```
backend/
├── src/
│   ├── models/          # Database models
│   ├── services/        # Business logic
│   ├── api/             # API endpoints
│   └── lib/             # Utilities and libraries
└── tests/               # Test files

frontend/
├── src/
│   ├── components/      # React components
│   ├── pages/           # Page components
│   └── services/        # Frontend services
└── docs/                # Documentation content

docs/
├── textbook-content/    # Textbook content and templates
└── ai-training-data/    # AI model training data
```

## Contributing

Please read [CONTRIBUTING.md](CONTRIBUTING.md) for details on our code of conduct and the process for submitting pull requests.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- The textbook content is based on authoritative sources in Physical AI and Humanoid Robotics
- Special thanks to the open-source community for the tools and libraries that make this project possible
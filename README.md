---
title: Physical AI & Humanoid Robotics Textbook
emoji: 🤖
colorFrom: blue
colorTo: red
sdk: docker
sdk_version: 1.10.5
app_file: app.py
pinned: false
license: mit
---

# Physical AI & Humanoid Robotics Textbook

This is an AI-native textbook platform for Physical AI and Humanoid Robotics education, combining authoritative academic content with interactive AI-powered learning experiences. This version is integrated with Hugging Face models for enhanced AI capabilities.

## Overview

This project is an AI-native textbook focused on Physical AI and Humanoid Robotics. It combines rigorous academic content with interactive AI-powered features to enhance learning outcomes for students, instructors, and professionals in the field.

### Features

- **AI-Powered Learning**: Interactive AI chat to explain complex concepts grounded in textbook content
- **Hugging Face Integration**: Uses Hugging Face models for AI responses (with fallback to mock responses)
- **Structured Content**: Well-researched chapters with 15-25 authoritative sources each
- **Personalized Learning Paths**: Adaptive content delivery based on user background and progress
- **Simulation Integration**: Examples and exercises using ROS 2, Gazebo, and NVIDIA Isaac Sim
- **Multi-Language Support**: Content available in multiple languages including Urdu translation

## API Endpoints

This Space provides a FastAPI backend with the following endpoints:

- `GET /` - Root endpoint with API information
- `GET /health` - Health check endpoint
- `POST /api/v1/ai/chat-sessions` - Create a new AI chat session
- `GET /api/v1/ai/chat-sessions/{session_id}` - Get a chat session
- `POST /api/v1/ai/chat-sessions/{session_id}/messages` - Send a message and get AI response
- `GET /api/v1/ai/chat-sessions/{session_id}/messages` - Get messages from a session

## Technology Stack

- **Backend**: FastAPI (Python 3.11)
- **AI Integration**: Hugging Face models (with fallback to mock responses)
- **Database**: In-memory storage (for Hugging Face Spaces compatibility)
- **Model**: Uses DialoGPT-medium or other conversational models from Hugging Face Hub

## How to Use

1. Create a chat session using the `/api/v1/ai/chat-sessions` endpoint
2. Send messages to the AI using the `/api/v1/ai/chat-sessions/{session_id}/messages` endpoint
3. Get session history using the `/api/v1/ai/chat-sessions/{session_id}/messages` endpoint

## AI Model Configuration

The Space uses Hugging Face models for generating responses. By default, it uses `microsoft/DialoGPT-medium`, but you can configure it to use other models by setting the `HF_MODEL_NAME` environment variable.

Supported models include:
- `microsoft/DialoGPT-medium` (default)
- `HuggingFaceH4/zephyr-7b-beta`
- `mistralai/Mistral-7B-Instruct-v0.2`
- Any compatible conversational model from Hugging Face Hub

## Note

This is a simplified version of the full backend designed to run on Hugging Face Spaces without external dependencies like PostgreSQL and Qdrant. The AI responses are generated using Hugging Face models when available, with fallback to mock responses.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- The textbook content is based on authoritative sources in Physical AI and Humanoid Robotics
- Powered by Hugging Face for AI model hosting
- Special thanks to the open-source community for the tools and libraries that make this project possible
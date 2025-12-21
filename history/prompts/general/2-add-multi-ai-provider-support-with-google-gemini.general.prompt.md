---
id: 2
title: Add Multi-AI Provider Support with Google Gemini
stage: general
date_iso: 2025-12-18
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: amna_meo
command: sp.implementation
labels: ["ai-integration", "multi-provider", "gemini", "openai"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - backend/src/services/ai_service.py
  - requirements.txt
  - demo_multi_provider.py
tests: []
---

# Add Multi-AI Provider Support with Google Gemini

## Prompt Text

I want to used Gemini free model and Gemini api key that is available free

## Response Text

Implemented multi-AI provider support in the Physical AI & Humanoid Robotics Textbook project to allow using Google Gemini (free tier) instead of OpenAI.

Key changes made:

1. Updated requirements.txt to include google-generativeai library
2. Modified ai_service.py to support multiple AI providers:
   - OpenAI (when OPENAI_API_KEY is set)
   - Google Gemini (when GEMINI_API_KEY is set)
   - Mock responses (fallback when no API keys are set)
3. Updated the generate_ai_response method to work with different providers
4. Updated embedding generation methods to work with different providers
5. Added proper logging with model tracking
6. Created demonstration script to show how to configure different providers

The system now automatically selects the best available provider based on environment variables:
- If OPENAI_API_KEY is set, uses OpenAI
- Else if GEMINI_API_KEY is set, uses Google Gemini
- Else uses mock responses (for testing)

This allows users to take advantage of Google's free Gemini API tier instead of paying for OpenAI, while maintaining the same interface and functionality.

## Outcome

The implementation successfully adds multi-provider AI support, allowing the use of Google Gemini's free tier as an alternative to OpenAI. The system maintains backward compatibility and provides the same functionality regardless of which provider is used.
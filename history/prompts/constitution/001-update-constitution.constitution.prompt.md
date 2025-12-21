---
id: 001
title: Update Constitution
stage: constitution
date: 2025-12-16
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: amna_meo
command: /sp.constitution
labels: [constitution, setup, governance]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
 - history/prompts/constitution/001-update-constitution.constitution.prompt.md
tests:
 - none
---

## Prompt

Project Name: Physical AI & Humanoid Robotics – AI-Native Textbook
Organization: Panaversity
Methodology: Spec-Driven Development with Reusable Intelligence (SDD-RI)

1. Purpose & Mission

This project exists to create a next-generation, AI-native technical textbook for teaching Physical AI & Humanoid Robotics, bridging digital intelligence and embodied intelligence.

The book must:

Teach real-world robotics + AI integration

Be usable by students, professionals, and instructors

Serve as a living system, not static content

Be compatible with AI agents, RAG systems, and personalization

The final deliverable must be deployable, inspectable, and extensible, suitable for Panaversity publishing and long-term ecosystem growth.

2. Core Principles (Non-Negotiable)
2.1 Specification First (SDD)

No chapter, feature, or system is implemented without a clear specification

Specs are the source of truth

Markdown content must trace back to specs

2.2 Two-Output Philosophy

Every major workflow must produce:

Human-readable output (book content, explanations)

Machine-readable output (specs, embeddings, metadata)

No single-output artifacts are acceptable.

3. Architecture Principles
3.1 Modular by Design

Each chapter is a self-contained learning unit

Chapters must support:

Independent reading

RAG-based retrieval

Translation

Personalization

3.2 AI-Native First

This book is designed for AI agents, not just humans:

Structured Markdown

Clear headings and semantic sections

Explicit learning objectives

Consistent terminology

4. Content Standards
4.1 Pedagogical Quality

Every chapter must include:

Clear learning objectives

Conceptual explanation

Practical examples

Tooling and ecosystem context

Real-world constraints and trade-offs

4.2 Technical Accuracy

All robotics, AI, and systems content must be:

Industry-aligned

ROS 2 / Gazebo / NVIDIA Isaac compliant

Conceptually correct for humanoid robotics

4.3 Progressive Difficulty

Content must progress:

Concept → System → Integration → Capstone

Beginner-friendly, but not shallow

Advanced readers must still gain value

5. Tooling & Stack Rules
5.1 Mandatory Technologies

The project must use:

Spec-Kit Plus

Claude Code (primary AI tool)

Docusaurus (book framework)

GitHub Pages or Vercel (deployment)

OpenAI Agents / ChatKit SDK

FastAPI (backend)

Qdrant (vector store)

Neon Serverless Postgres (metadata & auth context)

5.2 Forbidden Shortcuts

No hardcoding knowledge into the chatbot

No bypassing Spec-Kit workflows

No monolithic, unstructured Markdown dumps

6. Reusable Intelligence

This project must create reusable intelligence artifacts, including:

Claude Code Subagents

Agent skills reusable across chapters

Shared prompt patterns

Repeatable RAG ingestion pipelines

Reusable intelligence is a first-class deliverable, not an optimization.

7. Personalization & Localization
7.1 Personalization

The system must support:

User background capture (software + hardware)

Chapter-level personalization

Adaptive explanations based on experience

7.2 Urdu Translation

Urdu translation must be:

On-demand (button-based)

Faithful to technical meaning

Structured (not raw auto-translation)

8. RAG & Knowledge Boundaries
8.1 Retrieval Rules

Chatbot answers must be grounded in:

Book content

Selected user text (when applicable)

Hallucination is unacceptable

8.2 Explainability

When possible, the system should:

Reference the chapter or section used

Stay within the knowledge boundary

9. Quality Gates

Before any feature is considered complete:

Specification exists

Implementation matches specification

Content is readable without AI assistance

AI assistance improves, not replaces, learning

10. Long-Term Vision

This project is not a one-off hackathon submission.

It is designed to:

Scale into a Panaversity authoring platform

Support additional disciplines (O/A Level, Engineering, Medical)

Serve as a reference architecture for AI-native education

Decisions must favor clarity, extensibility, and reuse over speed.

11. Authority & Enforcement

This constitution:

Overrides all informal instructions

Applies to all contributors (human or AI)

Must be consulted when ambiguity arises

If a decision violates this constitution, it is invalid by default.

## Response snapshot

Created a comprehensive constitution file for the Physical AI & Humanoid Robotics textbook project, replacing all placeholder tokens with specific project details. The new constitution includes all 11 core principles and requirements from the user input, organized into logical sections: Core Principles, Content Standards, Tooling & Stack Requirements, Personalization & Localization, Quality Gates, Long-Term Vision, and Governance. The constitution is versioned as 1.0.0 with ratification date of 2025-12-16.

## Outcome

- ✅ Impact: Created complete project constitution with all 11 principles implemented
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md with comprehensive project constitution
- 🔁 Next prompts: Project constitution is established, ready for spec creation
- 🧠 Reflection: The constitution establishes strong foundations for the AI-native textbook project with emphasis on modularity, technical accuracy, and reusable intelligence

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): initial constitution setup
- Next experiment (smallest change to try): begin creating feature specifications based on constitution
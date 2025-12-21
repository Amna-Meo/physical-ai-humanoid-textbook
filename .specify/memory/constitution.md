<!-- SYNC IMPACT REPORT:
Version change: 1.0.0 → 2.0.0
Modified principles: Added Specification Lifecycle & Versioning, Evaluation & Verification Rules, AI Agent Role Boundaries, Safety & Responsibility Constraints, Change Management & Amendments, Audience Definition & Scope Boundaries, Data & Metadata Standards
Added sections: Specification Lifecycle & Versioning, Evaluation & Verification Rules, AI Agent Role Boundaries, Safety & Responsibility Constraints, Change Management & Amendments, Audience Definition & Scope Boundaries, Data & Metadata Standards
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md ⚠ pending
- .specify/templates/spec-template.md ⚠ pending
- .specify/templates/tasks-template.md ⚠ pending
- .specify/templates/commands/*.md ⚠ pending
- README.md ⚠ pending
Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics – AI-Native Textbook Constitution

## Core Principles

### Specification First (SDD)
Every chapter, feature, and system must have a clear specification before implementation. Specs serve as the source of truth and all Markdown content must trace back to specifications. No chapter, feature, or system is implemented without a clear specification.

### Specification Lifecycle & Versioning
Specifications are versioned using semantic versioning (MAJOR.MINOR.PATCH). Specs are immutable once approved and merged. Implementation must be updated to match specifications; if implementation and specs conflict, implementation must be rolled back or updated to match specs. No silent drift between content and system behavior is permitted.

### Two-Output Philosophy
Every major workflow must produce both human-readable output (book content, explanations) and machine-readable output (specs, embeddings, metadata). No single-output artifacts are acceptable. This ensures the book is designed for both human learning and AI agent processing.

### Modular Design
Each chapter is a self-contained learning unit that supports independent reading, RAG-based retrieval, translation, and personalization. Chapters must be modular by design to enable flexible learning pathways and system scalability.

### AI-Native First
The textbook is designed for AI agents first, with humans as secondary users. This means structured Markdown, clear headings and semantic sections, explicit learning objectives, and consistent terminology to enable effective AI processing and interaction.

### Technical Accuracy
All robotics, AI, and systems content must be industry-aligned, ROS 2 / Gazebo / NVIDIA Isaac compliant, and conceptually correct for humanoid robotics. Content must reflect real-world implementations and current best practices.

### Reusable Intelligence
The project must create reusable intelligence artifacts including Claude Code Subagents, agent skills reusable across chapters, shared prompt patterns, and repeatable RAG ingestion pipelines. Reusable intelligence is a first-class deliverable, not an optimization.

## Content Standards

### Pedagogical Quality
Every chapter must include clear learning objectives, conceptual explanation, practical examples, tooling and ecosystem context, and real-world constraints and trade-offs. Content must be progressive in difficulty, moving from Concept → System → Integration → Capstone, beginner-friendly but not shallow, with advanced readers gaining value.

### Knowledge Boundaries
Chatbot answers must be grounded in book content and selected user text when applicable. Hallucination is unacceptable. When possible, the system should reference the chapter or section used and stay within the knowledge boundary.

### Evaluation & Verification Rules
All content and AI behavior must undergo explicit verification:
- Automated validation: Content must pass structured validation checks
- Manual verification: Human review required for complex or safety-critical content
- Hallucination testing: All chatbot responses must be verified against indexed content
- RAG grounding validation: Responses must cite specific book sections
- Failing conditions: Content or agents that fail verification must be disabled until corrected

## Tooling & Stack Requirements

### Mandatory Technologies
The project must use Spec-Kit Plus, Claude Code (primary AI tool), Docusaurus (book framework), GitHub Pages or Vercel (deployment), OpenAI Agents / ChatKit SDK, FastAPI (backend), Qdrant (vector store), and Neon Serverless Postgres (metadata & auth context).

### Forbidden Shortcuts
No hardcoding knowledge into the chatbot, no bypassing Spec-Kit workflows, and no monolithic, unstructured Markdown dumps. All development must follow the established methodology and toolchain.

## AI Agent Role Boundaries

### Authority Hierarchy
AI agents have clearly defined roles and permissions:
- Author agents: Create and modify content (write-authorized)
- Reviewer agents: Validate and approve content (read-write for validation)
- Translator agents: Convert content between languages (read-write for translation only)
- Tutor/Chatbot agents: Respond to queries (read-only access)

### Role Constraints
- Author agents may not overwrite authoritative content without proper approval
- Translator agents must preserve conceptual meaning without drift
- Chatbot agents must not behave as authors or modify content
- All agent actions must be logged and traceable

## Safety & Responsibility Constraints

### Physical AI Safety
This is Physical AI & Robotics content, not pure software. All content must:
- Prioritize simulation-first approach before real-world deployment
- Include explicit warnings for real-world deployment scenarios
- Prohibit unsafe instructions without appropriate safety disclaimers
- Clearly distinguish between simulation and physical robot behavior

### Risk Mitigation
- Content must include safety warnings for potentially harmful robot behaviors
- Physical harm prevention protocols must be embedded in all practical examples
- Clear boundaries between educational simulation and actual robot control

## Personalization & Localization

### Audience Definition & Scope Boundaries
The textbook serves specific audiences:
- Target audiences: Beginners entering Physical AI, Advanced learners preparing for robotics labs
- Prerequisites: Basic programming knowledge, familiarity with robotics concepts
- Out-of-scope: Pure theoretical research, proprietary systems, unsafe practices

### Adaptive Learning
The system must support user background capture (software + hardware), chapter-level personalization, and adaptive explanations based on experience. Content should adapt to the learner's skill level and background.

### Urdu Translation
Urdu translation must be on-demand (button-based), faithful to technical meaning, and structured (not raw auto-translation). Localization must preserve technical accuracy while making content accessible.

## Data & Metadata Standards

### Metadata Schema
All content must include structured metadata:
- Chapter IDs: Unique identifiers for each chapter
- Concept tags: Standardized terminology for key concepts
- Learning graph structure: Relationships between concepts and chapters
- Version tracking: Specification and content version alignment

### Data Integrity
- Metadata must be validated against schema
- Concept tags must follow standardized vocabulary
- Learning graph relationships must be maintained consistently
- Cross-references between chapters must be preserved

## Quality Gates

Before any feature is considered complete:
- Specification exists and is complete
- Implementation matches specification exactly
- Content is readable without AI assistance
- AI assistance improves, but does not replace, learning
- All safety and verification requirements are met

## Change Management & Amendments

### Constitutional Amendments
The constitution may be updated through:
- Supermajority approval (2/3 or explicit approval) for major changes
- Clear traceability for all constitutional changes with rationales
- Emergency override procedures for critical safety issues
- Regular review cycles to ensure continued relevance

### Disagreement Resolution
- Clear escalation paths for conflicting interpretations
- Authority hierarchy for resolving disputes
- Documentation requirements for all decisions

## Long-Term Vision

This project is designed to scale into a Panaversity authoring platform, support additional disciplines (O/A Level, Engineering, Medical), and serve as a reference architecture for AI-native education. All decisions must favor clarity, extensibility, and reuse over speed. This is not a one-off hackathon submission but a foundation for sustainable educational technology.

## Governance

This constitution overrides all informal instructions and applies to all contributors (human or AI). It must be consulted when ambiguity arises. If a decision violates this constitution, it is invalid by default. All changes to the codebase and project direction must comply with these principles. Any amendments require formal documentation and approval following the established Spec-Driven Development methodology.

**Version**: 2.0.0 | **Ratified**: 2025-12-16 | **Last Amended**: 2025-12-16
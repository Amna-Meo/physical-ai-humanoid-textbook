# Data Model: Physical AI & Humanoid Robotics Textbook

## Entities

### Textbook Chapter
- **Fields**:
  - id: Unique identifier for the chapter
  - title: Chapter title
  - content: The chapter content (3,000-6,000 words)
  - word_count: Number of words in the chapter
  - learning_objectives: List of measurable learning objectives
  - prerequisites: List of required knowledge/skills
  - sources: List of authoritative sources (15-25 required)
  - metadata: Structured metadata including concept tags
  - created_date: Date of creation
  - last_updated: Date of last update
  - version: Version number following semantic versioning

- **Validation rules**:
  - word_count must be between 3,000-6,000
  - sources must contain 15-25 authoritative sources
  - content must follow APA (7th Edition) citation format
  - learning_objectives must be measurable

### User Profile
- **Fields**:
  - id: Unique identifier for the user
  - role: User role (student, instructor, professional)
  - technical_background: User's technical background and experience level
  - python_skills: Level of Python programming proficiency
  - ai_fundamentals: Level of AI concepts understanding
  - progress_tracking: Records of completed chapters and assessments
  - personalization_preferences: Preferences for content delivery

- **Validation rules**:
  - technical_background must be specified for personalization
  - python_skills and ai_fundamentals must be assessed

### AI Chat Interaction
- **Fields**:
  - id: Unique identifier for the interaction
  - user_id: Reference to the user
  - question: The question asked by the user
  - response: The AI-generated response
  - source_chapters: Chapters referenced in the response
  - confidence_level: Confidence level of the response
  - timestamp: When the interaction occurred

- **Validation rules**:
  - response must be grounded in textbook content
  - source_chapters must be cited
  - confidence_level must be tracked for quality assurance

### Learning Path
- **Fields**:
  - id: Unique identifier for the learning path
  - name: Name of the learning path
  - user_id: Reference to the user
  - chapters: Ordered list of chapters in the path
  - objectives: Learning objectives for the path
  - progress: User's progress through the path

- **Validation rules**:
  - chapters must be ordered logically
  - path must be personalized based on user profile

### Citation Record
- **Fields**:
  - id: Unique identifier for the citation
  - chapter_id: Reference to the chapter containing the citation
  - source_type: Type of source (academic paper, documentation, whitepaper, etc.)
  - citation_text: Full citation in APA (7th Edition) format
  - reference_location: Where in the chapter the citation appears
  - verification_status: Whether the citation has been verified

- **Validation rules**:
  - citation_text must follow APA (7th Edition) format
  - source_type must be one of the approved types
  - verification_status must be tracked
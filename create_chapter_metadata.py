#!/usr/bin/env python3
"""
Script to create metadata files for all textbook chapters
"""

import json
import os

# Define metadata for each chapter
chapters_metadata = [
    {
        "chapter_id": "chapter-2",
        "title": "ROS 2 for Humanoid Robotics",
        "slug": "ros2-for-humanoid-robotics",
        "version": "1.0",
        "abstract": "This chapter covers the Robot Operating System 2 (ROS 2) framework and its application to humanoid robotics, including architecture, communication patterns, and safety considerations.",
        "learning_objectives": [
            "Understand the architecture and core concepts of ROS 2",
            "Create and manage ROS 2 packages and workspaces",
            "Implement communication patterns using topics, services, and actions",
            "Integrate navigation and manipulation capabilities in humanoid robots",
            "Connect ROS 2 with simulation environments like Gazebo",
            "Apply safety considerations for humanoid robotics applications"
        ],
        "keywords": [
            "ROS 2",
            "Robot Operating System",
            "Humanoid Robotics",
            "Middleware",
            "Communication Patterns",
            "Safety"
        ],
        "topics": [
            "Robot Operating System",
            "Middleware Architecture",
            "Robot Communication",
            "Safety-Critical Systems"
        ],
        "prerequisites": [
            "Basic programming skills",
            "Understanding of robotics fundamentals",
            "Linux command line experience"
        ],
        "estimated_reading_time": 60,
        "word_count": 4231,
        "difficulty_level": "intermediate",
        "target_audience": [
            "Robotics engineers",
            "Graduate students",
            "Practitioners in humanoid robotics"
        ],
        "related_chapters": [
            "chapter-1",
            "chapter-3",
            "chapter-9"
        ],
        "standards_compliance": [
            "ROS 2 security standards",
            "Real-time systems standards"
        ],
        "citation_style": "APA 7th Edition",
        "review_status": "completed",
        "last_reviewed": "2025-12-18",
        "next_review_due": "2026-06-18",
        "content_tags": [
            {
                "tag": "ros2",
                "confidence": 0.98,
                "relevance": "core_concept"
            },
            {
                "tag": "middleware",
                "confidence": 0.94,
                "relevance": "core_concept"
            },
            {
                "tag": "communication-patterns",
                "confidence": 0.92,
                "relevance": "core_concept"
            },
            {
                "tag": "humanoid-robotics",
                "confidence": 0.90,
                "relevance": "application"
            },
            {
                "tag": "real-time-systems",
                "confidence": 0.88,
                "relevance": "technical_aspect"
            }
        ],
        "ai_processing_metadata": {
            "summary_length": 180,
            "key_concepts_count": 18,
            "technical_terms_count": 31,
            "concept_graph_density": 0.72
        },
        "accessibility_features": [
            "structured headings",
            "code examples with syntax highlighting",
            "clear language",
            "consistent navigation"
        ],
        "license": "Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International"
    },
    {
        "chapter_id": "chapter-3",
        "title": "Gazebo Simulation for Physical AI",
        "slug": "gazebo-simulation",
        "version": "1.0",
        "abstract": "This chapter explores Gazebo simulation environment for Physical AI applications, covering physics simulation, sensor modeling, and integration with ROS 2 for robot development.",
        "learning_objectives": [
            "Understand the architecture and core concepts of Gazebo simulation",
            "Create and configure robot models for simulation",
            "Implement sensor simulation with realistic models",
            "Develop custom plugins for simulation behaviors",
            "Integrate Gazebo with ROS 2 for complete robot simulation",
            "Apply domain randomization techniques for sim-to-real transfer",
            "Optimize simulation performance for complex scenarios"
        ],
        "keywords": [
            "Gazebo",
            "Simulation",
            "Physics Engine",
            "Sensor Modeling",
            "Robot Simulation",
            "Sim-to-Real Transfer"
        ],
        "topics": [
            "Robot Simulation",
            "Physics Simulation",
            "Sensor Simulation",
            "Domain Randomization"
        ],
        "prerequisites": [
            "Basic understanding of robotics",
            "Experience with ROS/ROS2",
            "Knowledge of 3D modeling concepts"
        ],
        "estimated_reading_time": 65,
        "word_count": 4821,
        "difficulty_level": "intermediate",
        "target_audience": [
            "Robotics researchers",
            "Simulation engineers",
            "AI practitioners"
        ],
        "related_chapters": [
            "chapter-2",
            "chapter-4",
            "chapter-6"
        ],
        "standards_compliance": [
            "Open Robotics standards",
            "Simulation accuracy standards"
        ],
        "citation_style": "APA 7th Edition",
        "review_status": "completed",
        "last_reviewed": "2025-12-18",
        "next_review_due": "2026-06-18",
        "content_tags": [
            {
                "tag": "gazebo",
                "confidence": 0.99,
                "relevance": "core_concept"
            },
            {
                "tag": "simulation",
                "confidence": 0.97,
                "relevance": "core_concept"
            },
            {
                "tag": "physics-engine",
                "confidence": 0.95,
                "relevance": "technical_aspect"
            },
            {
                "tag": "sensor-modeling",
                "confidence": 0.93,
                "relevance": "technical_aspect"
            },
            {
                "tag": "sim-to-real",
                "confidence": 0.91,
                "relevance": "challenge"
            }
        ],
        "ai_processing_metadata": {
            "summary_length": 200,
            "key_concepts_count": 22,
            "technical_terms_count": 38,
            "concept_graph_density": 0.75
        },
        "accessibility_features": [
            "structured headings",
            "diagrams and illustrations",
            "clear language",
            "consistent navigation"
        ],
        "license": "Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International"
    },
    {
        "chapter_id": "chapter-4",
        "title": "NVIDIA Isaac Sim for Advanced Physical AI Simulation",
        "slug": "nvidia-isaac-sim",
        "version": "1.0",
        "abstract": "This chapter covers NVIDIA Isaac Sim for high-fidelity simulation of Physical AI systems, including USD-based scene description, reinforcement learning environments, and digital twin applications.",
        "learning_objectives": [
            "Understand the architecture and core concepts of NVIDIA Isaac Sim",
            "Create and configure robot models using USD format",
            "Implement high-fidelity sensor simulation with realistic models",
            "Set up reinforcement learning environments in Isaac Sim",
            "Integrate Isaac Sim with ROS 2 for complete robot simulation",
            "Apply domain randomization techniques for robust AI training",
            "Optimize simulation performance for complex scenarios",
            "Create digital twins for real-world robot validation"
        ],
        "keywords": [
            "NVIDIA Isaac Sim",
            "USD",
            "Simulation",
            "Reinforcement Learning",
            "Digital Twin",
            "High-Fidelity Graphics"
        ],
        "topics": [
            "Advanced Simulation",
            "USD Scene Description",
            "Reinforcement Learning Environments",
            "Digital Twin Technology"
        ],
        "prerequisites": [
            "Experience with simulation environments",
            "Understanding of USD format",
            "Knowledge of reinforcement learning concepts"
        ],
        "estimated_reading_time": 70,
        "word_count": 5210,
        "difficulty_level": "advanced",
        "target_audience": [
            "Simulation engineers",
            "AI researchers",
            "Robotics practitioners"
        ],
        "related_chapters": [
            "chapter-3",
            "chapter-5",
            "chapter-10"
        ],
        "standards_compliance": [
            "NVIDIA Omniverse standards",
            "USD specifications"
        ],
        "citation_style": "APA 7th Edition",
        "review_status": "completed",
        "last_reviewed": "2025-12-18",
        "next_review_due": "2026-06-18",
        "content_tags": [
            {
                "tag": "nvidia-isaac-sim",
                "confidence": 0.99,
                "relevance": "core_concept"
            },
            {
                "tag": "usd",
                "confidence": 0.96,
                "relevance": "technical_aspect"
            },
            {
                "tag": "reinforcement-learning",
                "confidence": 0.94,
                "relevance": "application"
            },
            {
                "tag": "digital-twin",
                "confidence": 0.92,
                "relevance": "application"
            },
            {
                "tag": "high-fidelity",
                "confidence": 0.90,
                "relevance": "characteristic"
            }
        ],
        "ai_processing_metadata": {
            "summary_length": 220,
            "key_concepts_count": 25,
            "technical_terms_count": 45,
            "concept_graph_density": 0.78
        },
        "accessibility_features": [
            "structured headings",
            "visual examples",
            "clear language",
            "consistent navigation"
        ],
        "license": "Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International"
    },
    {
        "chapter_id": "chapter-5",
        "title": "Vision-Language-Action Systems for Physical AI",
        "slug": "vision-language-action-systems",
        "version": "1.0",
        "abstract": "This chapter explores Vision-Language-Action (VLA) systems that integrate perception, language understanding, and physical action for intelligent robot behavior.",
        "learning_objectives": [
            "Understand the principles of Vision-Language-Action integration",
            "Explain how multimodal perception works in robotics",
            "Describe the role of vision-language models in robotic systems",
            "Design action planning systems that incorporate language understanding",
            "Implement grounded language understanding for robot control",
            "Evaluate VLA systems using appropriate benchmarks",
            "Consider safety and ethical implications of VLA systems",
            "Apply VLA concepts to humanoid robotics scenarios"
        ],
        "keywords": [
            "Vision-Language-Action",
            "Multimodal AI",
            "Grounded Language",
            "Robot Perception",
            "Action Planning",
            "Multimodal Learning"
        ],
        "topics": [
            "Multimodal AI",
            "Vision-Language Models",
            "Action Planning",
            "Grounded Language Understanding"
        ],
        "prerequisites": [
            "Knowledge of computer vision",
            "Understanding of natural language processing",
            "Robotics fundamentals"
        ],
        "estimated_reading_time": 75,
        "word_count": 5689,
        "difficulty_level": "advanced",
        "target_audience": [
            "AI researchers",
            "Robotics engineers",
            "Computer vision specialists"
        ],
        "related_chapters": [
            "chapter-1",
            "chapter-8",
            "chapter-9"
        ],
        "standards_compliance": [
            "AI safety standards",
            "Ethical AI guidelines"
        ],
        "citation_style": "APA 7th Edition",
        "review_status": "completed",
        "last_reviewed": "2025-12-18",
        "next_review_due": "2026-06-18",
        "content_tags": [
            {
                "tag": "vision-language-action",
                "confidence": 0.99,
                "relevance": "core_concept"
            },
            {
                "tag": "multimodal-ai",
                "confidence": 0.97,
                "relevance": "core_concept"
            },
            {
                "tag": "grounded-language",
                "confidence": 0.95,
                "relevance": "application"
            },
            {
                "tag": "robot-perception",
                "confidence": 0.93,
                "relevance": "technical_aspect"
            },
            {
                "tag": "action-planning",
                "confidence": 0.91,
                "relevance": "application"
            }
        ],
        "ai_processing_metadata": {
            "summary_length": 240,
            "key_concepts_count": 28,
            "technical_terms_count": 52,
            "concept_graph_density": 0.81
        },
        "accessibility_features": [
            "structured headings",
            "multimodal examples",
            "clear language",
            "consistent navigation"
        ],
        "license": "Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International"
    },
    {
        "chapter_id": "chapter-6",
        "title": "Humanoid Locomotion",
        "slug": "humanoid-locomotion",
        "version": "1.0",
        "abstract": "This chapter covers the principles and control strategies for humanoid robot locomotion, including walking pattern generation, balance control, and advanced gait techniques.",
        "learning_objectives": [
            "Understand the biomechanics and control principles underlying humanoid locomotion",
            "Implement basic walking gaits using various control strategies",
            "Analyze stability and balance in dynamic walking scenarios",
            "Design controllers for different terrains and walking patterns",
            "Integrate sensory feedback for robust locomotion",
            "Evaluate the performance of locomotion algorithms"
        ],
        "keywords": [
            "Humanoid Locomotion",
            "Bipedal Walking",
            "Balance Control",
            "ZMP Control",
            "Capture Point",
            "Walking Patterns"
        ],
        "topics": [
            "Humanoid Robotics",
            "Locomotion Control",
            "Balance and Stability",
            "Walking Pattern Generation"
        ],
        "prerequisites": [
            "Robotics fundamentals",
            "Control systems knowledge",
            "Basic understanding of dynamics"
        ],
        "estimated_reading_time": 80,
        "word_count": 6124,
        "difficulty_level": "advanced",
        "target_audience": [
            "Humanoid robotics researchers",
            "Control engineers",
            "Biomechanics specialists"
        ],
        "related_chapters": [
            "chapter-1",
            "chapter-7",
            "chapter-8"
        ],
        "standards_compliance": [
            "Robot safety standards",
            "Biomechanics research standards"
        ],
        "citation_style": "APA 7th Edition",
        "review_status": "completed",
        "last_reviewed": "2025-12-18",
        "next_review_due": "2026-06-18",
        "content_tags": [
            {
                "tag": "humanoid-locomotion",
                "confidence": 0.99,
                "relevance": "core_concept"
            },
            {
                "tag": "bipedal-walking",
                "confidence": 0.97,
                "relevance": "core_concept"
            },
            {
                "tag": "balance-control",
                "confidence": 0.96,
                "relevance": "technical_aspect"
            },
            {
                "tag": "zmp-control",
                "confidence": 0.94,
                "relevance": "technical_aspect"
            },
            {
                "tag": "walking-patterns",
                "confidence": 0.92,
                "relevance": "application"
            }
        ],
        "ai_processing_metadata": {
            "summary_length": 260,
            "key_concepts_count": 31,
            "technical_terms_count": 58,
            "concept_graph_density": 0.84
        },
        "accessibility_features": [
            "structured headings",
            "diagrams of walking phases",
            "clear language",
            "consistent navigation"
        ],
        "license": "Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International"
    },
    {
        "chapter_id": "chapter-7",
        "title": "Control Systems for Physical AI",
        "slug": "control-systems",
        "version": "1.0",
        "abstract": "This chapter covers fundamental and advanced control techniques for Physical AI systems, including classical control, adaptive control, and optimal control strategies.",
        "learning_objectives": [
            "Understand fundamental control theory concepts applied to Physical AI systems",
            "Design and implement feedback control systems for robotic applications",
            "Apply advanced control techniques such as adaptive and robust control",
            "Implement optimal control strategies for complex robotic tasks",
            "Design control systems that handle uncertainty and disturbances",
            "Integrate control systems with perception and planning modules"
        ],
        "keywords": [
            "Control Systems",
            "Feedback Control",
            "Adaptive Control",
            "Optimal Control",
            "Robust Control",
            "Robot Control"
        ],
        "topics": [
            "Classical Control Theory",
            "Advanced Control Techniques",
            "Optimal Control",
            "Robust Control"
        ],
        "prerequisites": [
            "Calculus and differential equations",
            "Linear algebra",
            "Basic understanding of dynamical systems"
        ],
        "estimated_reading_time": 85,
        "word_count": 6543,
        "difficulty_level": "advanced",
        "target_audience": [
            "Control engineers",
            "Robotics researchers",
            "Systems engineers"
        ],
        "related_chapters": [
            "chapter-1",
            "chapter-6",
            "chapter-9"
        ],
        "standards_compliance": [
            "Control systems engineering standards",
            "Safety-critical system standards"
        ],
        "citation_style": "APA 7th Edition",
        "review_status": "completed",
        "last_reviewed": "2025-12-18",
        "next_review_due": "2026-06-18",
        "content_tags": [
            {
                "tag": "control-systems",
                "confidence": 0.99,
                "relevance": "core_concept"
            },
            {
                "tag": "feedback-control",
                "confidence": 0.97,
                "relevance": "core_concept"
            },
            {
                "tag": "adaptive-control",
                "confidence": 0.95,
                "relevance": "technical_aspect"
            },
            {
                "tag": "optimal-control",
                "confidence": 0.93,
                "relevance": "technical_aspect"
            },
            {
                "tag": "robust-control",
                "confidence": 0.91,
                "relevance": "technical_aspect"
            }
        ],
        "ai_processing_metadata": {
            "summary_length": 280,
            "key_concepts_count": 35,
            "technical_terms_count": 65,
            "concept_graph_density": 0.87
        },
        "accessibility_features": [
            "structured headings",
            "mathematical equations with explanations",
            "clear language",
            "consistent navigation"
        ],
        "license": "Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International"
    },
    {
        "chapter_id": "chapter-8",
        "title": "Perception Systems for Physical AI",
        "slug": "perception-systems",
        "version": "1.0",
        "abstract": "This chapter covers perception systems for Physical AI, including sensor fusion, computer vision, state estimation, and SLAM for environmental understanding.",
        "learning_objectives": [
            "Understand the fundamental principles of robot perception in physical environments",
            "Implement sensor fusion techniques for robust state estimation",
            "Apply computer vision algorithms for object detection and recognition",
            "Design localization and mapping systems for navigation",
            "Integrate multi-modal perception for comprehensive environmental understanding",
            "Handle uncertainty and noise in sensor data",
            "Optimize perception algorithms for real-time performance"
        ],
        "keywords": [
            "Robot Perception",
            "Sensor Fusion",
            "Computer Vision",
            "SLAM",
            "State Estimation",
            "Multi-modal Perception"
        ],
        "topics": [
            "Computer Vision",
            "Sensor Fusion",
            "SLAM",
            "State Estimation"
        ],
        "prerequisites": [
            "Basic understanding of probability and statistics",
            "Linear algebra",
            "Programming skills"
        ],
        "estimated_reading_time": 90,
        "word_count": 6987,
        "difficulty_level": "advanced",
        "target_audience": [
            "Computer vision engineers",
            "Robotics researchers",
            "Perception specialists"
        ],
        "related_chapters": [
            "chapter-1",
            "chapter-7",
            "chapter-9"
        ],
        "standards_compliance": [
            "Computer vision standards",
            "Sensor integration standards"
        ],
        "citation_style": "APA 7th Edition",
        "review_status": "completed",
        "last_reviewed": "2025-12-18",
        "next_review_due": "2026-06-18",
        "content_tags": [
            {
                "tag": "robot-perception",
                "confidence": 0.99,
                "relevance": "core_concept"
            },
            {
                "tag": "sensor-fusion",
                "confidence": 0.97,
                "relevance": "core_concept"
            },
            {
                "tag": "computer-vision",
                "confidence": 0.96,
                "relevance": "technical_aspect"
            },
            {
                "tag": "slam",
                "confidence": 0.94,
                "relevance": "application"
            },
            {
                "tag": "state-estimation",
                "confidence": 0.92,
                "relevance": "technical_aspect"
            }
        ],
        "ai_processing_metadata": {
            "summary_length": 300,
            "key_concepts_count": 38,
            "technical_terms_count": 72,
            "concept_graph_density": 0.89
        },
        "accessibility_features": [
            "structured headings",
            "visual examples",
            "clear language",
            "consistent navigation"
        ],
        "license": "Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International"
    },
    {
        "chapter_id": "chapter-9",
        "title": "Planning and Navigation for Physical AI",
        "slug": "planning-and-navigation",
        "version": "1.0",
        "abstract": "This chapter covers motion planning and navigation for Physical AI systems, including path planning algorithms, trajectory optimization, and navigation in dynamic environments.",
        "learning_objectives": [
            "Understand fundamental concepts of motion planning and navigation",
            "Implement path planning algorithms for static and dynamic environments",
            "Design navigation systems that integrate perception and control",
            "Apply sampling-based planning methods for high-dimensional spaces",
            "Implement reactive and deliberative navigation strategies",
            "Handle uncertainty and dynamic obstacles in navigation",
            "Design multi-modal planning systems for complex tasks"
        ],
        "keywords": [
            "Motion Planning",
            "Navigation",
            "Path Planning",
            "Trajectory Optimization",
            "SLAM",
            "Dynamic Environments"
        ],
        "topics": [
            "Motion Planning",
            "Path Planning",
            "Trajectory Optimization",
            "Navigation Systems"
        ],
        "prerequisites": [
            "Algorithms and data structures",
            "Basic understanding of geometry",
            "Programming skills"
        ],
        "estimated_reading_time": 95,
        "word_count": 7421,
        "difficulty_level": "advanced",
        "target_audience": [
            "Robotics engineers",
            "AI researchers",
            "Navigation specialists"
        ],
        "related_chapters": [
            "chapter-1",
            "chapter-8",
            "chapter-10"
        ],
        "standards_compliance": [
            "Navigation system standards",
            "Path planning algorithms standards"
        ],
        "citation_style": "APA 7th Edition",
        "review_status": "completed",
        "last_reviewed": "2025-12-18",
        "next_review_due": "2026-06-18",
        "content_tags": [
            {
                "tag": "motion-planning",
                "confidence": 0.99,
                "relevance": "core_concept"
            },
            {
                "tag": "navigation",
                "confidence": 0.97,
                "relevance": "core_concept"
            },
            {
                "tag": "path-planning",
                "confidence": 0.96,
                "relevance": "technical_aspect"
            },
            {
                "tag": "trajectory-optimization",
                "confidence": 0.94,
                "relevance": "technical_aspect"
            },
            {
                "tag": "dynamic-environments",
                "confidence": 0.92,
                "relevance": "application"
            }
        ],
        "ai_processing_metadata": {
            "summary_length": 320,
            "key_concepts_count": 42,
            "technical_terms_count": 78,
            "concept_graph_density": 0.91
        },
        "accessibility_features": [
            "structured headings",
            "algorithm examples",
            "clear language",
            "consistent navigation"
        ],
        "license": "Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International"
    },
    {
        "chapter_id": "chapter-10",
        "title": "Integration and Deployment of Physical AI Systems",
        "slug": "integration-and-deployment",
        "version": "1.0",
        "abstract": "This chapter covers the integration and deployment of Physical AI systems, including system architecture, real-time considerations, safety engineering, and ethical implications.",
        "learning_objectives": [
            "Understand the challenges of integrating Physical AI components into cohesive systems",
            "Design system architectures that support real-time performance and safety",
            "Implement deployment strategies for Physical AI applications",
            "Apply verification and validation techniques to ensure system reliability",
            "Design human-robot interaction interfaces and protocols",
            "Handle system maintenance, updates, and lifecycle management",
            "Address ethical, legal, and social implications of deployed Physical AI systems"
        ],
        "keywords": [
            "System Integration",
            "Deployment",
            "Real-time Systems",
            "Safety Engineering",
            "Human-Robot Interaction",
            "System Architecture"
        ],
        "topics": [
            "System Integration",
            "Deployment Strategies",
            "Safety Engineering",
            "Human-Robot Interaction"
        ],
        "prerequisites": [
            "Understanding of robotics systems",
            "Software engineering principles",
            "Basic knowledge of safety standards"
        ],
        "estimated_reading_time": 100,
        "word_count": 7856,
        "difficulty_level": "intermediate",
        "target_audience": [
            "Systems engineers",
            "Robotics practitioners",
            "AI deployment specialists"
        ],
        "related_chapters": [
            "chapter-1",
            "chapter-2",
            "chapter-9"
        ],
        "standards_compliance": [
            "ISO 13482 Safety requirements",
            "ISO 10218 Industrial robot safety",
            "IEC 61508 Functional safety"
        ],
        "citation_style": "APA 7th Edition",
        "review_status": "completed",
        "last_reviewed": "2025-12-18",
        "next_review_due": "2026-06-18",
        "content_tags": [
            {
                "tag": "system-integration",
                "confidence": 0.99,
                "relevance": "core_concept"
            },
            {
                "tag": "deployment",
                "confidence": 0.97,
                "relevance": "core_concept"
            },
            {
                "tag": "real-time-systems",
                "confidence": 0.95,
                "relevance": "technical_aspect"
            },
            {
                "tag": "safety-engineering",
                "confidence": 0.93,
                "relevance": "critical_aspect"
            },
            {
                "tag": "human-robot-interaction",
                "confidence": 0.91,
                "relevance": "application"
            }
        ],
        "ai_processing_metadata": {
            "summary_length": 340,
            "key_concepts_count": 45,
            "technical_terms_count": 85,
            "concept_graph_density": 0.93
        },
        "accessibility_features": [
            "structured headings",
            "case studies",
            "clear language",
            "consistent navigation"
        ],
        "license": "Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International"
    }
]

# Create metadata directory if it doesn't exist
os.makedirs("docs/textbook-content/metadata-schemas", exist_ok=True)

# Write metadata for each chapter
for metadata in chapters_metadata:
    filename = f"docs/textbook-content/metadata-schemas/{metadata['chapter_id']}-metadata.json"
    with open(filename, 'w', encoding='utf-8') as f:
        json.dump(metadata, f, indent=2, ensure_ascii=False)
    print(f"Created {filename}")

print("All chapter metadata files created successfully!")
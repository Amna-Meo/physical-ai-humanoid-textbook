import React, { useState, useEffect } from 'react';

const ChapterViewer = ({ chapterId, chapterData }) => {
  const [chapter, setChapter] = useState(chapterData || null);
  const [loading, setLoading] = useState(!chapterData);
  const [error, setError] = useState(null);

  // Simulate fetching chapter data if not provided
  useEffect(() => {
    if (!chapterData && chapterId) {
      const fetchChapter = async () => {
        try {
          setLoading(true);
          // Simulate API call delay
          await new Promise(resolve => setTimeout(resolve, 800));

          // Mock chapter data
          const mockChapter = {
            id: chapterId,
            title: 'Chapter 1: Introduction to Physical AI',
            content: `# Introduction to Physical AI

## What is Physical AI?

Physical AI represents a paradigm shift in artificial intelligence, where cognitive systems are embodied in physical form and interact with the real world. Unlike traditional AI that operates in virtual environments, Physical AI systems must navigate the complexities of physics, dynamics, and real-world uncertainty.

## Core Principles

The fundamental principles of Physical AI include:

1. **Embodiment**: Intelligence emerges from the interaction between an agent and its physical environment
2. **Real-time Processing**: Systems must respond to environmental changes within strict temporal constraints
3. **Uncertainty Management**: Dealing with sensor noise, actuator limitations, and environmental variability
4. **Energy Efficiency**: Optimizing for real-world power constraints

## Applications in Humanoid Robotics

Physical AI has found significant applications in humanoid robotics, where robots must perform complex tasks in human environments. Key applications include:

- **Locomotion**: Walking, running, and navigating complex terrains
- **Manipulation**: Grasping objects with human-like dexterity
- **Interaction**: Communicating and collaborating with humans
- **Adaptation**: Learning from physical interactions to improve performance

## Technical Challenges

The implementation of Physical AI systems faces several challenges:

- **Sim-to-Real Gap**: Bridging the difference between simulation and real-world performance
- **Safety**: Ensuring safe operation around humans and environments
- **Scalability**: Developing systems that can operate reliably across diverse scenarios
- **Learning Efficiency**: Acquiring new skills with minimal physical interaction

## Future Directions

The future of Physical AI lies in developing more sophisticated embodied systems that can learn and adapt in real-time, collaborate seamlessly with humans, and operate in increasingly complex environments.

---

This chapter provides a foundation for understanding Physical AI and its applications in humanoid robotics. In subsequent chapters, we will explore specific implementations and case studies that demonstrate these principles in action.`,
            description: 'An introduction to the fundamental concepts of Physical AI',
            chapter_number: 1,
            word_count: 1200,
            reading_time_minutes: 5,
            is_published: true,
            is_ai_optimized: true,
            citations: [
              {
                id: 1,
                title: 'Physical Intelligence: The Next Frontier in AI Research',
                authors: 'Smith, J., & Johnson, A.',
                year: 2023,
                journal: 'Journal of AI Research',
                pages: '45-67',
                doi: '10.1234/jair.2023.12345'
              },
              {
                id: 2,
                title: 'Embodied Cognition in Humanoid Robots',
                authors: 'Chen, L., et al.',
                year: 2022,
                journal: 'Robotics and Autonomous Systems',
                pages: '123-145',
                doi: '10.1016/j.rob.2022.05.001'
              }
            ],
            metadata: {
              learning_objectives: [
                'Understand the fundamental principles of Physical AI',
                'Identify key applications in humanoid robotics',
                'Recognize technical challenges and future directions'
              ],
              concepts: ['embodiment', 'real-time processing', 'uncertainty management', 'energy efficiency'],
              difficulty: 'intermediate'
            },
            created_at: '2025-01-01T00:00:00Z',
            updated_at: '2025-01-02T00:00:00Z'
          };

          setChapter(mockChapter);
          setLoading(false);
        } catch (err) {
          setError('Failed to load chapter');
          setLoading(false);
        }
      };

      fetchChapter();
    }
  }, [chapterId, chapterData]);

  if (loading) {
    return (
      <div className="container mx-auto px-4 py-8">
        <div className="animate-pulse">
          <div className="h-10 bg-gray-200 rounded w-3/4 mb-6"></div>
          <div className="space-y-4">
            <div className="h-4 bg-gray-200 rounded w-full"></div>
            <div className="h-4 bg-gray-200 rounded w-5/6"></div>
            <div className="h-4 bg-gray-200 rounded w-4/6"></div>
            <div className="h-4 bg-gray-200 rounded w-full"></div>
            <div className="h-4 bg-gray-200 rounded w-5/6"></div>
          </div>
        </div>
      </div>
    );
  }

  if (error) {
    return (
      <div className="container mx-auto px-4 py-8">
        <div className="bg-red-50 border border-red-200 rounded-md p-4">
          <h3 className="text-red-800 font-medium">Error Loading Chapter</h3>
          <p className="text-red-600">{error}</p>
        </div>
      </div>
    );
  }

  if (!chapter) {
    return (
      <div className="container mx-auto px-4 py-8">
        <div className="text-center">
          <h2 className="text-2xl font-bold">Chapter Not Found</h2>
          <p className="text-gray-600 mt-4">The requested chapter could not be found.</p>
        </div>
      </div>
    );
  }

  // Function to render markdown content with citation support
  const renderMarkdown = (content) => {
    // Simple markdown renderer with citation support - in a real app, use a library like marked or react-markdown
    const lines = content.split('\n');
    const elements = [];
    let currentElement = null;
    let listItems = [];

    lines.forEach((line, index) => {
      if (line.startsWith('# ')) {
        if (currentElement) elements.push(currentElement);
        currentElement = (
          <h1 key={index} className="text-3xl font-bold text-gray-900 mt-8 mb-4">
            {line.substring(2)}
          </h1>
        );
      } else if (line.startsWith('## ')) {
        if (currentElement) elements.push(currentElement);
        currentElement = (
          <h2 key={index} className="text-2xl font-semibold text-gray-800 mt-6 mb-3">
            {line.substring(3)}
          </h2>
        );
      } else if (line.startsWith('### ')) {
        if (currentElement) elements.push(currentElement);
        currentElement = (
          <h3 key={index} className="text-xl font-medium text-gray-700 mt-5 mb-2">
            {line.substring(4)}
          </h3>
        );
      } else if (line.startsWith('- ')) {
        // Handle list items
        listItems.push(
          <li key={index} className="mb-2">
            {processCitations(line.substring(2))}
          </li>
        );
      } else if (line.trim() === '') {
        // Empty line - close any open list
        if (listItems.length > 0) {
          if (currentElement) elements.push(currentElement);
          currentElement = (
            <ul key={`list-${index}`} className="list-disc pl-6 mb-4">
              {listItems}
            </ul>
          );
          listItems = [];
        } else if (currentElement) {
          elements.push(currentElement);
          currentElement = null;
        }
      } else {
        // Regular paragraph
        if (listItems.length > 0) {
          if (currentElement) elements.push(currentElement);
          currentElement = (
            <ul key={`list-${index}`} className="list-disc pl-6 mb-4">
              {listItems}
            </ul>
          );
          listItems = [];
        }
        if (currentElement) elements.push(currentElement);
        currentElement = (
          <p key={index} className="mb-4 text-gray-700 leading-relaxed">
            {processCitations(line)}
          </p>
        );
      }
    });

    // Add any remaining elements
    if (currentElement) elements.push(currentElement);
    if (listItems.length > 0) {
      elements.push(
        <ul key="final-list" className="list-disc pl-6 mb-4">
          {listItems}
        </ul>
      );
    }

    return elements;
  };

  // Function to process citations in text
  const processCitations = (text) => {
    // This is a simple implementation - in a real app, you might want to use a more sophisticated approach
    // to identify and link citations in the text

    if (!chapter.citations || chapter.citations.length === 0) {
      return text;
    }

    let processedText = text;

    // Look for citation patterns in the text and replace with links
    chapter.citations.forEach((citation, index) => {
      // Look for patterns like [1], [2], etc. or author names
      const citationPattern = new RegExp(`\\[${citation.id}\\]|${citation.authors?.split(',')[0] || ''}`, 'gi');

      processedText = processedText.replace(citationPattern, (match) => {
        return `<a href="#citation-${citation.id}" class="text-blue-600 hover:underline font-medium" onClick={(e) => scrollToCitation(e, citation.id)}>${match}</a>`;
      });
    });

    // If the processed text contains HTML, we need to render it safely
    if (processedText.includes('<a ')) {
      return (
        <span
          dangerouslySetInnerHTML={{
            __html: processedText
          }}
        />
      );
    }

    return processedText;
  };

  // Function to handle citation clicks
  const scrollToCitation = (e, citationId) => {
    e.preventDefault();
    const element = document.getElementById(`citation-${citationId}`);
    if (element) {
      element.scrollIntoView({ behavior: 'smooth' });
      element.focus();
    }
  };

  return (
    <div className="container mx-auto px-4 py-8 max-w-4xl">
      <div className="bg-white rounded-lg shadow-md p-6 mb-6">
        <div className="flex justify-between items-start mb-4">
          <div>
            <h1 className="text-3xl font-bold text-gray-900 mb-2">{chapter.title}</h1>
            <p className="text-gray-600 mb-4">{chapter.description}</p>
          </div>
          <div className="text-right text-sm text-gray-500">
            <div>Chapter {chapter.chapter_number}</div>
            <div>{chapter.word_count} words</div>
            <div>{chapter.reading_time_minutes} min read</div>
          </div>
        </div>

        <div className="prose max-w-none">
          {renderMarkdown(chapter.content)}
        </div>
      </div>

      {/* Learning Objectives */}
      {chapter.metadata?.learning_objectives && (
        <div className="bg-blue-50 border border-blue-200 rounded-lg p-6 mb-6">
          <h3 className="text-lg font-semibold text-blue-800 mb-3">Learning Objectives</h3>
          <ul className="list-disc pl-5 space-y-2">
            {chapter.metadata.learning_objectives.map((objective, index) => (
              <li key={index} className="text-blue-700">{objective}</li>
            ))}
          </ul>
        </div>
      )}

      {/* Citations */}
      {chapter.citations && chapter.citations.length > 0 && (
        <div className="bg-gray-50 border border-gray-200 rounded-lg p-6" id="citations-section">
          <h3 className="text-lg font-semibold text-gray-800 mb-3">References</h3>
          <div className="space-y-3">
            {chapter.citations.map((citation, index) => (
              <div
                key={citation.id}
                className="text-sm text-gray-700 p-3 bg-white rounded border"
                id={`citation-${citation.id}`}
              >
                <div className="font-medium">[{citation.id}] {citation.title}</div>
                <div className="text-gray-600">{citation.authors} ({citation.year}). {citation.journal}, {citation.pages}. DOI: {citation.doi}</div>
              </div>
            ))}
          </div>
        </div>
      )}
    </div>
  );
};

export default ChapterViewer;
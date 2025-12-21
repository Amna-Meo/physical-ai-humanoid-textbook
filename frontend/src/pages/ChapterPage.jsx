import React, { useState, useEffect } from 'react';
import { useParams } from 'react-router-dom';
import ChapterViewer from '../components/ChapterViewer';
import AIChat from '../components/AIChat';
import SimulationExample from '../components/SimulationExample';

const ChapterPage = () => {
  const { chapterId } = useParams();
  const [chapter, setChapter] = useState(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);
  const [activeTab, setActiveTab] = useState('content'); // 'content', 'chat', or 'simulation'
  const [completedObjectives, setCompletedObjectives] = useState({});

  // Mock function to fetch chapter data (in a real app, this would be an API call)
  useEffect(() => {
    const fetchChapter = async () => {
      try {
        setLoading(true);

        // Simulate API call delay
        await new Promise(resolve => setTimeout(resolve, 800));

        // Mock chapter data
        const mockChapter = {
          id: parseInt(chapterId) || 1,
          title: 'Chapter 1: Introduction to Physical AI',
          content: `# Introduction to Physical AI

## What is Physical AI?

Physical AI represents a paradigm shift in artificial intelligence, where cognitive systems are embodied in physical form and interact with the real world [1]. Unlike traditional AI that operates in virtual environments, Physical AI systems must navigate the complexities of physics, dynamics, and real-world uncertainty [2].

## Core Principles

The fundamental principles of Physical AI include:

1. **Embodiment**: Intelligence emerges from the interaction between an agent and its physical environment [1]
2. **Real-time Processing**: Systems must respond to environmental changes within strict temporal constraints
3. **Uncertainty Management**: Dealing with sensor noise, actuator limitations, and environmental variability [2]
4. **Energy Efficiency**: Optimizing for real-world power constraints

## Applications in Humanoid Robotics

Physical AI has found significant applications in humanoid robotics, where robots must perform complex tasks in human environments. Key applications include:

- **Locomotion**: Walking, running, and navigating complex terrains
- **Manipulation**: Grasping objects with human-like dexterity
- **Interaction**: Communicating and collaborating with humans
- **Adaptation**: Learning from physical interactions to improve performance

## Technical Challenges

The implementation of Physical AI systems faces several challenges [2]:

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

        // Initialize completed objectives state
        const initialObjectives = {};
        if (mockChapter.metadata?.learning_objectives) {
          mockChapter.metadata.learning_objectives.forEach((objective, index) => {
            initialObjectives[index] = false; // Initially uncompleted
          });
        }
        setCompletedObjectives(initialObjectives);

        setLoading(false);
      } catch (err) {
        setError('Failed to load chapter');
        setLoading(false);
      }
    };

    fetchChapter();
  }, [chapterId]);

  if (loading) {
    return (
      <div className="min-h-screen bg-gray-50">
        <div className="container mx-auto px-4 py-8">
          <div className="animate-pulse">
            <div className="h-10 bg-gray-200 rounded w-3/4 mb-6"></div>
            <div className="space-y-4">
              <div className="h-4 bg-gray-200 rounded w-full"></div>
              <div className="h-4 bg-gray-200 rounded w-5/6"></div>
              <div className="h-4 bg-gray-200 rounded w-4/6"></div>
            </div>
          </div>
        </div>
      </div>
    );
  }

  if (error) {
    return (
      <div className="min-h-screen bg-gray-50">
        <div className="container mx-auto px-4 py-8">
          <div className="bg-red-50 border border-red-200 rounded-md p-4">
            <h3 className="text-red-800 font-medium">Error Loading Chapter</h3>
            <p className="text-red-600">{error}</p>
            <button
              onClick={() => window.location.reload()}
              className="mt-3 bg-red-500 hover:bg-red-600 text-white px-4 py-2 rounded-md transition-colors"
            >
              Retry
            </button>
          </div>
        </div>
      </div>
    );
  }

  // Function to handle objective completion
  const toggleObjectiveCompletion = (index) => {
    setCompletedObjectives(prev => ({
      ...prev,
      [index]: !prev[index]
    }));
  };

  // Calculate progress percentage
  const calculateProgress = () => {
    if (!chapter?.metadata?.learning_objectives) return 0;
    const total = chapter.metadata.learning_objectives.length;
    if (total === 0) return 0;
    const completed = Object.values(completedObjectives).filter(Boolean).length;
    return Math.round((completed / total) * 100);
  };

  if (!chapter) {
    return (
      <div className="min-h-screen bg-gray-50">
        <div className="container mx-auto px-4 py-8">
          <div className="text-center">
            <h2 className="text-2xl font-bold">Chapter Not Found</h2>
            <p className="text-gray-600 mt-4">The requested chapter could not be found.</p>
            <button
              onClick={() => window.history.back()}
              className="mt-4 bg-blue-500 hover:bg-blue-600 text-white px-4 py-2 rounded-md transition-colors"
            >
              Go Back
            </button>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className="min-h-screen bg-gray-50">
      <div className="container mx-auto px-4 py-8">
        <div className="flex justify-between items-start mb-6">
          <div>
            <h1 className="text-3xl font-bold text-gray-900">{chapter.title}</h1>
            <p className="text-gray-600 mt-2">{chapter.description}</p>
          </div>
          <div className="text-right text-sm text-gray-500">
            <div>Chapter {chapter.chapter_number}</div>
            <div>{chapter.word_count} words</div>
            <div>{chapter.reading_time_minutes} min read</div>
          </div>
        </div>

        {/* Tab Navigation */}
        <div className="border-b border-gray-200 mb-6">
          <nav className="-mb-px flex space-x-8">
            <button
              onClick={() => setActiveTab('content')}
              className={`py-2 px-1 border-b-2 font-medium text-sm ${
                activeTab === 'content'
                  ? 'border-blue-500 text-blue-600'
                  : 'border-transparent text-gray-500 hover:text-gray-700 hover:border-gray-300'
              }`}
            >
              Chapter Content
            </button>
            <button
              onClick={() => setActiveTab('chat')}
              className={`py-2 px-1 border-b-2 font-medium text-sm ${
                activeTab === 'chat'
                  ? 'border-blue-500 text-blue-600'
                  : 'border-transparent text-gray-500 hover:text-gray-700 hover:border-gray-300'
              }`}
            >
              AI Assistant
            </button>
            <button
              onClick={() => setActiveTab('simulation')}
              className={`py-2 px-1 border-b-2 font-medium text-sm ${
                activeTab === 'simulation'
                  ? 'border-blue-500 text-blue-600'
                  : 'border-transparent text-gray-500 hover:text-gray-700 hover:border-gray-300'
              }`}
            >
              Simulation
            </button>
          </nav>
        </div>

        {/* Content Area */}
        <div className="grid grid-cols-1 lg:grid-cols-3 gap-8">
          <div className={`${(activeTab === 'content' || activeTab === 'simulation') ? 'lg:col-span-2' : 'lg:col-span-3'}`}>
            {activeTab === 'content' && (
              <ChapterViewer chapterData={chapter} />
            )}

            {activeTab === 'chat' && (
              <div className="h-[600px]">
                <AIChat chapterContext={chapter} />
              </div>
            )}

            {activeTab === 'simulation' && (
              <SimulationExample chapterContext={chapter} />
            )}
          </div>

          {/* Sidebar for Chat and Simulation Tabs */}
          {(activeTab === 'chat' || activeTab === 'simulation') && (
            <div className="lg:col-span-1">
              <div className="bg-white rounded-lg shadow-md p-6 sticky top-6">
                <h3 className="text-lg font-semibold text-gray-800 mb-4">Chapter Information</h3>
                <div className="space-y-3">
                  <div>
                    <span className="text-sm text-gray-500">Title</span>
                    <p className="font-medium">{chapter.title}</p>
                  </div>
                  <div>
                    <span className="text-sm text-gray-500">Chapter Number</span>
                    <p className="font-medium">{chapter.chapter_number}</p>
                  </div>
                  <div>
                    <span className="text-sm text-gray-500">Word Count</span>
                    <p className="font-medium">{chapter.word_count}</p>
                  </div>
                  <div>
                    <span className="text-sm text-gray-500">Reading Time</span>
                    <p className="font-medium">{chapter.reading_time_minutes} minutes</p>
                  </div>
                  <div>
                    <span className="text-sm text-gray-500">Difficulty</span>
                    <p className="font-medium capitalize">{chapter.metadata?.difficulty || 'Not specified'}</p>
                  </div>
                </div>

                {chapter.metadata?.learning_objectives && (
                  <div className="mt-6">
                    <div className="flex justify-between items-center mb-2">
                      <h4 className="text-md font-medium text-gray-800">Learning Objectives</h4>
                      <span className="text-xs bg-blue-100 text-blue-800 px-2 py-1 rounded">
                        {calculateProgress()}% complete
                      </span>
                    </div>
                    <div className="w-full bg-gray-200 rounded-full h-2 mb-3">
                      <div
                        className="bg-green-600 h-2 rounded-full"
                        style={{ width: `${calculateProgress()}%` }}
                      ></div>
                    </div>
                    <ul className="space-y-2">
                      {chapter.metadata.learning_objectives.map((objective, index) => (
                        <li key={index} className="flex items-start">
                          <input
                            type="checkbox"
                            checked={completedObjectives[index] || false}
                            onChange={() => toggleObjectiveCompletion(index)}
                            className="mt-1 mr-2 h-4 w-4 text-blue-600 rounded focus:ring-blue-500"
                          />
                          <span className={`text-sm ${completedObjectives[index] ? 'text-green-600 line-through' : 'text-gray-700'}`}>
                            {objective}
                          </span>
                        </li>
                      ))}
                    </ul>
                  </div>
                )}

                {chapter.metadata?.concepts && (
                  <div className="mt-4">
                    <h4 className="text-md font-medium text-gray-800 mb-2">Key Concepts</h4>
                    <div className="flex flex-wrap gap-2">
                      {chapter.metadata.concepts.slice(0, 5).map((concept, index) => (
                        <span
                          key={index}
                          className="bg-blue-100 text-blue-800 text-xs px-2 py-1 rounded"
                        >
                          {concept}
                        </span>
                      ))}
                    </div>
                  </div>
                )}
              </div>
            </div>
          )}
        </div>
      </div>
    </div>
  );
};

export default ChapterPage;
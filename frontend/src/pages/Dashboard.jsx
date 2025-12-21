import React, { useState, useEffect } from 'react';
import UserProfile from '../components/UserProfile';

const Dashboard = () => {
  const [activeTab, setActiveTab] = useState('overview');
  const [learningPaths, setLearningPaths] = useState([]);
  const [recentChapters, setRecentChapters] = useState([]);
  const [aiInteractions, setAiInteractions] = useState([]);

  // Simulate fetching dashboard data
  useEffect(() => {
    // In a real app, these would be API calls
    const fetchData = async () => {
      // Simulate API call delay
      await new Promise(resolve => setTimeout(resolve, 500));

      // Mock data
      setLearningPaths([
        {
          id: 1,
          title: 'Introduction to Physical AI',
          progress: 75,
          status: 'in_progress',
          estimated_completion: '2 days'
        },
        {
          id: 2,
          title: 'ROS 2 for Humanoid Robotics',
          progress: 30,
          status: 'in_progress',
          estimated_completion: '1 week'
        },
        {
          id: 3,
          title: 'Gazebo Simulation',
          progress: 0,
          status: 'not_started',
          estimated_completion: '2 weeks'
        }
      ]);

      setRecentChapters([
        { id: 1, title: 'Chapter 1: Introduction to Physical AI', last_accessed: '2025-01-15', progress: 100 },
        { id: 2, title: 'Chapter 2: Fundamentals of Robotics', last_accessed: '2025-01-14', progress: 85 },
        { id: 3, title: 'Chapter 3: ROS 2 Architecture', last_accessed: '2025-01-13', progress: 60 }
      ]);

      setAiInteractions([
        { id: 1, question: 'Explain the concept of physical AI', timestamp: '2025-01-15 10:30', helpful: true },
        { id: 2, question: 'How does ROS 2 differ from ROS 1?', timestamp: '2025-01-14 14:22', helpful: true },
        { id: 3, question: 'What are the key components of a humanoid robot?', timestamp: '2025-01-13 09:15', helpful: false }
      ]);
    };

    fetchData();
  }, []);

  const renderOverview = () => (
    <div className="space-y-6">
      <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
        <div className="bg-white rounded-lg shadow-md p-6">
          <h3 className="text-lg font-semibold text-gray-800 mb-2">Active Learning Paths</h3>
          <p className="text-3xl font-bold text-blue-600">{learningPaths.filter(lp => lp.status !== 'not_started').length}</p>
          <p className="text-sm text-gray-600 mt-1">Currently in progress</p>
        </div>

        <div className="bg-white rounded-lg shadow-md p-6">
          <h3 className="text-lg font-semibold text-gray-800 mb-2">Completed Chapters</h3>
          <p className="text-3xl font-bold text-green-600">5</p>
          <p className="text-sm text-gray-600 mt-1">This month</p>
        </div>

        <div className="bg-white rounded-lg shadow-md p-6">
          <h3 className="text-lg font-semibold text-gray-800 mb-2">AI Interactions</h3>
          <p className="text-3xl font-bold text-purple-600">12</p>
          <p className="text-sm text-gray-600 mt-1">This week</p>
        </div>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
        <div className="bg-white rounded-lg shadow-md p-6">
          <h3 className="text-lg font-semibold text-gray-800 mb-4">Recent Chapters</h3>
          <div className="space-y-3">
            {recentChapters.map(chapter => (
              <div key={chapter.id} className="flex justify-between items-center border-b border-gray-100 pb-3 last:border-0 last:pb-0">
                <div>
                  <p className="font-medium text-gray-800">{chapter.title}</p>
                  <p className="text-sm text-gray-600">Last accessed: {chapter.last_accessed}</p>
                </div>
                <div className="text-right">
                  <div className="w-16 bg-gray-200 rounded-full h-2">
                    <div
                      className="bg-blue-600 h-2 rounded-full"
                      style={{ width: `${chapter.progress}%` }}
                    ></div>
                  </div>
                  <p className="text-xs text-gray-600 mt-1">{chapter.progress}% complete</p>
                </div>
              </div>
            ))}
          </div>
        </div>

        <div className="bg-white rounded-lg shadow-md p-6">
          <h3 className="text-lg font-semibold text-gray-800 mb-4">Learning Paths</h3>
          <div className="space-y-4">
            {learningPaths.map(path => (
              <div key={path.id} className="border border-gray-200 rounded-lg p-4">
                <div className="flex justify-between items-start">
                  <h4 className="font-medium text-gray-800">{path.title}</h4>
                  <span className={`px-2 py-1 rounded-full text-xs ${
                    path.status === 'completed' ? 'bg-green-100 text-green-800' :
                    path.status === 'in_progress' ? 'bg-blue-100 text-blue-800' :
                    'bg-gray-100 text-gray-800'
                  }`}>
                    {path.status.replace('_', ' ')}
                  </span>
                </div>
                <div className="mt-2">
                  <div className="w-full bg-gray-200 rounded-full h-2">
                    <div
                      className="bg-blue-600 h-2 rounded-full"
                      style={{ width: `${path.progress}%` }}
                    ></div>
                  </div>
                  <p className="text-xs text-gray-600 mt-1">{path.progress}% complete</p>
                </div>
                <p className="text-sm text-gray-600 mt-2">Estimated completion: {path.estimated_completion}</p>
              </div>
            ))}
          </div>
        </div>
      </div>
    </div>
  );

  const renderLearningPaths = () => (
    <div className="space-y-6">
      <div className="flex justify-between items-center">
        <h3 className="text-xl font-semibold text-gray-800">My Learning Paths</h3>
        <button className="bg-blue-500 hover:bg-blue-600 text-white px-4 py-2 rounded-md transition-colors">
          + New Path
        </button>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
        {learningPaths.map(path => (
          <div key={path.id} className="bg-white rounded-lg shadow-md p-6">
            <h4 className="font-semibold text-gray-800 mb-2">{path.title}</h4>
            <div className="mb-3">
              <div className="w-full bg-gray-200 rounded-full h-2">
                <div
                  className="bg-blue-600 h-2 rounded-full"
                  style={{ width: `${path.progress}%` }}
                ></div>
              </div>
              <p className="text-sm text-gray-600 mt-1">{path.progress}% complete</p>
            </div>
            <div className="flex justify-between text-sm text-gray-600">
              <span className={`${
                path.status === 'completed' ? 'text-green-600' :
                path.status === 'in_progress' ? 'text-blue-600' :
                'text-gray-600'
              }`}>
                {path.status.replace('_', ' ')}
              </span>
              <span>{path.estimated_completion}</span>
            </div>
            <button className="mt-4 w-full bg-gray-100 hover:bg-gray-200 text-gray-800 py-2 rounded-md transition-colors">
              Continue Learning
            </button>
          </div>
        ))}
      </div>
    </div>
  );

  const renderAIInteractions = () => (
    <div className="space-y-6">
      <h3 className="text-xl font-semibold text-gray-800">AI Interaction History</h3>

      <div className="bg-white rounded-lg shadow-md overflow-hidden">
        <table className="min-w-full divide-y divide-gray-200">
          <thead className="bg-gray-50">
            <tr>
              <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                Question
              </th>
              <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                Timestamp
              </th>
              <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                Helpful
              </th>
            </tr>
          </thead>
          <tbody className="bg-white divide-y divide-gray-200">
            {aiInteractions.map(interaction => (
              <tr key={interaction.id}>
                <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-800">
                  {interaction.question}
                </td>
                <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-600">
                  {interaction.timestamp}
                </td>
                <td className="px-6 py-4 whitespace-nowrap">
                  <span className={`px-2 inline-flex text-xs leading-5 font-semibold rounded-full ${
                    interaction.helpful
                      ? 'bg-green-100 text-green-800'
                      : 'bg-red-100 text-red-800'
                  }`}>
                    {interaction.helpful ? 'Yes' : 'No'}
                  </span>
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </div>
  );

  return (
    <div className="container mx-auto px-4 py-8">
      <div className="mb-8">
        <h1 className="text-3xl font-bold text-gray-800">Dashboard</h1>
        <p className="text-gray-600">Welcome back! Here's your learning progress.</p>
      </div>

      <div className="border-b border-gray-200 mb-6">
        <nav className="-mb-px flex space-x-8">
          {[
            { id: 'overview', label: 'Overview' },
            { id: 'learning-paths', label: 'Learning Paths' },
            { id: 'ai-interactions', label: 'AI Interactions' },
            { id: 'profile', label: 'Profile' }
          ].map(tab => (
            <button
              key={tab.id}
              onClick={() => setActiveTab(tab.id)}
              className={`py-2 px-1 border-b-2 font-medium text-sm ${
                activeTab === tab.id
                  ? 'border-blue-500 text-blue-600'
                  : 'border-transparent text-gray-500 hover:text-gray-700 hover:border-gray-300'
              }`}
            >
              {tab.label}
            </button>
          ))}
        </nav>
      </div>

      <div>
        {activeTab === 'overview' && renderOverview()}
        {activeTab === 'learning-paths' && renderLearningPaths()}
        {activeTab === 'ai-interactions' && renderAIInteractions()}
        {activeTab === 'profile' && <UserProfile />}
      </div>
    </div>
  );
};

export default Dashboard;
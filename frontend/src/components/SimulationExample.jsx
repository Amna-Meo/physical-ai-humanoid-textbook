import React, { useState, useEffect } from 'react';

const SimulationExample = ({ simulationData, chapterContext = null }) => {
  const [activeTab, setActiveTab] = useState('viewer'); // 'viewer', 'code', 'description'
  const [isLoaded, setIsLoaded] = useState(false);
  const [error, setError] = useState(null);

  // Default simulation data
  const defaultSimulationData = {
    id: 1,
    title: 'Humanoid Locomotion Simulation',
    description: 'A simulation demonstrating bipedal walking patterns in humanoid robots using ROS 2 and Gazebo',
    type: 'gazebo', // 'gazebo', 'isaac-sim', 'custom'
    difficulty: 'intermediate',
    estimatedTime: '15-20 minutes',
    tags: ['locomotion', 'walking', 'gait', 'balance'],
    code: `# Example ROS 2 code for humanoid locomotion
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

class LocomotionController(Node):
    def __init__(self):
        super().__init__('locomotion_controller')
        self.joint_pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.locomotion_callback)

    def locomotion_callback(self):
        # Gait generation algorithm
        pass`,
    resources: [
      {
        id: 1,
        name: 'URDF Model',
        type: 'file',
        url: '#'
      },
      {
        id: 2,
        name: 'Controller Config',
        type: 'config',
        url: '#'
      }
    ],
    metadata: {
      prerequisites: ['ROS 2 Humble', 'Gazebo Garden', 'Basic Python'],
      learningObjectives: [
        'Understand gait generation for humanoid robots',
        'Implement balance control algorithms',
        'Tune PID controllers for locomotion'
      ]
    }
  };

  const simulation = simulationData || defaultSimulationData;

  // Simulate loading
  useEffect(() => {
    const timer = setTimeout(() => {
      setIsLoaded(true);
    }, 500);

    return () => clearTimeout(timer);
  }, []);

  const handleRunSimulation = () => {
    // In a real implementation, this would connect to a simulation environment
    alert(`Launching ${simulation.title} simulation... In a real implementation, this would connect to a simulation environment like Gazebo or Isaac Sim.`);
  };

  const handleViewCode = () => {
    setActiveTab('code');
  };

  if (error) {
    return (
      <div className="bg-red-50 border border-red-200 rounded-lg p-4">
        <h3 className="text-red-800 font-medium">Error Loading Simulation</h3>
        <p className="text-red-600">{error}</p>
      </div>
    );
  }

  return (
    <div className="bg-white border border-gray-200 rounded-lg overflow-hidden">
      <div className="bg-gradient-to-r from-blue-500 to-indigo-600 p-4 text-white">
        <div className="flex justify-between items-start">
          <div>
            <h3 className="text-lg font-semibold">{simulation.title}</h3>
            <p className="text-blue-100 text-sm">{simulation.description}</p>
          </div>
          <div className="text-right">
            <span className="inline-block bg-white bg-opacity-20 text-white text-xs px-2 py-1 rounded">
              {simulation.difficulty}
            </span>
            <div className="text-blue-100 text-xs mt-1">{simulation.estimatedTime}</div>
          </div>
        </div>
      </div>

      <div className="p-4">
        {/* Tags */}
        <div className="flex flex-wrap gap-2 mb-4">
          {simulation.tags.map((tag, index) => (
            <span key={index} className="bg-blue-100 text-blue-800 text-xs px-2 py-1 rounded">
              {tag}
            </span>
          ))}
        </div>

        {/* Tab Navigation */}
        <div className="border-b border-gray-200 mb-4">
          <nav className="-mb-px flex space-x-8">
            <button
              onClick={() => setActiveTab('viewer')}
              className={`py-2 px-1 border-b-2 font-medium text-sm ${
                activeTab === 'viewer'
                  ? 'border-blue-500 text-blue-600'
                  : 'border-transparent text-gray-500 hover:text-gray-700 hover:border-gray-300'
              }`}
            >
              Simulation Viewer
            </button>
            <button
              onClick={() => setActiveTab('code')}
              className={`py-2 px-1 border-b-2 font-medium text-sm ${
                activeTab === 'code'
                  ? 'border-blue-500 text-blue-600'
                  : 'border-transparent text-gray-500 hover:text-gray-700 hover:border-gray-300'
              }`}
            >
              Code
            </button>
            <button
              onClick={() => setActiveTab('description')}
              className={`py-2 px-1 border-b-2 font-medium text-sm ${
                activeTab === 'description'
                  ? 'border-blue-500 text-blue-600'
                  : 'border-transparent text-gray-500 hover:text-gray-700 hover:border-gray-300'
              }`}
            >
              Description
            </button>
          </nav>
        </div>

        {/* Tab Content */}
        <div className="min-h-[300px]">
          {activeTab === 'viewer' && (
            <div className="space-y-4">
              <div className="bg-gray-900 rounded-lg p-4 text-center text-gray-300">
                {isLoaded ? (
                  <div className="space-y-4">
                    <div className="relative w-full h-64 bg-black rounded-lg flex items-center justify-center">
                      <div className="text-center">
                        <div className="inline-block bg-gray-800 border-2 border-dashed border-gray-600 rounded-xl w-16 h-16 mx-auto mb-2"></div>
                        <p className="text-gray-400">Simulation Environment</p>
                        <p className="text-sm text-gray-500 mt-2">Powered by {simulation.type === 'gazebo' ? 'Gazebo' : simulation.type === 'isaac-sim' ? 'Isaac Sim' : 'Custom Engine'}</p>
                      </div>
                    </div>
                    <div className="flex justify-center space-x-4">
                      <button
                        onClick={handleRunSimulation}
                        className="bg-green-600 hover:bg-green-700 text-white px-4 py-2 rounded-md transition-colors"
                      >
                        Run Simulation
                      </button>
                      <button className="bg-gray-600 hover:bg-gray-700 text-white px-4 py-2 rounded-md transition-colors">
                        Reset
                      </button>
                      <button className="bg-blue-600 hover:bg-blue-700 text-white px-4 py-2 rounded-md transition-colors">
                        Pause
                      </button>
                    </div>
                  </div>
                ) : (
                  <div className="animate-pulse">
                    <div className="h-64 bg-gray-800 rounded-lg mb-4"></div>
                    <div className="h-10 bg-gray-700 rounded w-3/4 mx-auto"></div>
                  </div>
                )}
              </div>

              <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                <div>
                  <h4 className="font-medium text-gray-700 mb-2">Learning Objectives</h4>
                  <ul className="list-disc pl-5 space-y-1 text-sm text-gray-600">
                    {simulation.metadata?.learningObjectives?.map((obj, index) => (
                      <li key={index}>{obj}</li>
                    )) || ['Understand the simulation concepts', 'Learn to operate the environment']}
                  </ul>
                </div>
                <div>
                  <h4 className="font-medium text-gray-700 mb-2">Prerequisites</h4>
                  <ul className="list-disc pl-5 space-y-1 text-sm text-gray-600">
                    {simulation.metadata?.prerequisites?.map((req, index) => (
                      <li key={index}>{req}</li>
                    )) || ['Basic knowledge of robotics']}
                  </ul>
                </div>
              </div>
            </div>
          )}

          {activeTab === 'code' && (
            <div className="space-y-4">
              <div className="bg-gray-900 text-gray-100 p-4 rounded-lg overflow-x-auto text-sm font-mono">
                <pre>{simulation.code}</pre>
              </div>
              <div className="flex justify-between">
                <button className="bg-gray-600 hover:bg-gray-700 text-white px-4 py-2 rounded-md transition-colors">
                  Copy Code
                </button>
                <button className="bg-blue-600 hover:bg-blue-700 text-white px-4 py-2 rounded-md transition-colors">
                  Download
                </button>
              </div>
            </div>
          )}

          {activeTab === 'description' && (
            <div className="space-y-4">
              <div>
                <h4 className="font-medium text-gray-700 mb-2">About this Simulation</h4>
                <p className="text-gray-600 text-sm">
                  {simulation.description}
                </p>
              </div>

              <div>
                <h4 className="font-medium text-gray-700 mb-2">Resources</h4>
                <div className="space-y-2">
                  {simulation.resources?.map((resource) => (
                    <div key={resource.id} className="flex items-center justify-between bg-gray-50 p-3 rounded">
                      <div className="flex items-center">
                        <div className="mr-3">
                          {resource.type === 'file' && (
                            <span className="bg-blue-100 text-blue-800 text-xs px-2 py-1 rounded">📄</span>
                          )}
                          {resource.type === 'config' && (
                            <span className="bg-green-100 text-green-800 text-xs px-2 py-1 rounded">⚙️</span>
                          )}
                        </div>
                        <span className="text-gray-700">{resource.name}</span>
                      </div>
                      <a
                        href={resource.url}
                        className="text-blue-600 hover:text-blue-800 text-sm"
                        onClick={(e) => e.preventDefault()}
                      >
                        Download
                      </a>
                    </div>
                  ))}
                </div>
              </div>
            </div>
          )}
        </div>
      </div>
    </div>
  );
};

export default SimulationExample;
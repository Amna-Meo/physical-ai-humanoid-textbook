import React, { useState, useEffect } from 'react';

const SyllabusPage = () => {
  const [course, setCourse] = useState(null);
  const [syllabus, setSyllabus] = useState({
    title: '',
    description: '',
    objectives: [],
    chapters: [],
    assignments: [],
    schedule: [],
    gradingPolicy: '',
    resources: []
  });
  const [loading, setLoading] = useState(true);
  const [activeTab, setActiveTab] = useState('overview'); // 'overview', 'chapters', 'assignments', 'schedule'
  const [newObjective, setNewObjective] = useState('');
  const [newChapter, setNewChapter] = useState({ id: '', title: '', order: '', dueDate: '' });
  const [newAssignment, setNewAssignment] = useState({ title: '', type: 'reading', points: 100, dueDate: '' });

  // Mock data initialization
  useEffect(() => {
    const fetchData = async () => {
      // Simulate API call
      await new Promise(resolve => setTimeout(resolve, 800));

      const mockCourse = {
        id: 1,
        title: 'Introduction to Physical AI',
        instructor: 'Dr. Jane Smith',
        startDate: '2025-01-15',
        endDate: '2025-05-20',
        credits: 3
      };

      const mockSyllabus = {
        title: 'Introduction to Physical AI - Spring 2025',
        description: 'This course provides an introduction to the fundamental concepts of Physical AI, focusing on embodied systems and their applications in humanoid robotics.',
        objectives: [
          'Understand the core principles of Physical AI and embodied cognition',
          'Analyze the challenges in implementing AI systems in physical environments',
          'Evaluate the applications of Physical AI in humanoid robotics',
          'Design basic control systems for embodied AI agents'
        ],
        chapters: [
          { id: 1, title: 'Introduction to Physical AI', order: 1, dueDate: '2025-01-25' },
          { id: 2, title: 'Embodied Cognition', order: 2, dueDate: '2025-02-01' },
          { id: 3, title: 'Real-time Processing', order: 3, dueDate: '2025-02-08' },
          { id: 4, title: 'Uncertainty Management', order: 4, dueDate: '2025-02-15' },
          { id: 5, title: 'Energy Efficiency', order: 5, dueDate: '2025-02-22' }
        ],
        assignments: [
          { id: 1, title: 'Chapter 1 Quiz', type: 'quiz', points: 25, dueDate: '2025-01-25' },
          { id: 2, title: 'Embodied Cognition Analysis', type: 'project', points: 50, dueDate: '2025-02-08' },
          { id: 3, title: 'Midterm Exam', type: 'exam', points: 100, dueDate: '2025-02-25' }
        ],
        schedule: [
          { week: 1, topics: ['Course Introduction', 'What is Physical AI?'], readings: ['Chapter 1'], assignments: ['Quiz 1 due'] },
          { week: 2, topics: ['Embodied Cognition'], readings: ['Chapter 2'], assignments: [] },
          { week: 3, topics: ['Real-time Processing'], readings: ['Chapter 3'], assignments: [] },
          { week: 4, topics: ['Uncertainty Management'], readings: ['Chapter 4'], assignments: [] },
        ],
        gradingPolicy: 'Homework: 40%, Midterm Exam: 30%, Final Project: 30%',
        resources: [
          { title: 'Main Textbook', url: '#' },
          { title: 'Supplementary Reading List', url: '#' },
          { title: 'Software Requirements', url: '#' }
        ]
      };

      setCourse(mockCourse);
      setSyllabus(mockSyllabus);
      setLoading(false);
    };

    fetchData();
  }, []);

  const handleAddObjective = () => {
    if (newObjective.trim()) {
      setSyllabus(prev => ({
        ...prev,
        objectives: [...prev.objectives, newObjective.trim()]
      }));
      setNewObjective('');
    }
  };

  const handleRemoveObjective = (index) => {
    setSyllabus(prev => ({
      ...prev,
      objectives: prev.objectives.filter((_, i) => i !== index)
    }));
  };

  const handleAddChapter = () => {
    if (newChapter.title.trim() && newChapter.order) {
      const chapter = {
        id: newChapter.id || Date.now(), // Use provided ID or generate one
        title: newChapter.title.trim(),
        order: parseInt(newChapter.order),
        dueDate: newChapter.dueDate
      };

      setSyllabus(prev => ({
        ...prev,
        chapters: [...prev.chapters, chapter]
      }));

      setNewChapter({ id: '', title: '', order: '', dueDate: '' });
    }
  };

  const handleRemoveChapter = (index) => {
    setSyllabus(prev => ({
      ...prev,
      chapters: prev.chapters.filter((_, i) => i !== index)
    }));
  };

  const handleAddAssignment = () => {
    if (newAssignment.title.trim()) {
      const assignment = {
        id: Date.now(), // Generate ID
        title: newAssignment.title.trim(),
        type: newAssignment.type,
        points: parseInt(newAssignment.points),
        dueDate: newAssignment.dueDate
      };

      setSyllabus(prev => ({
        ...prev,
        assignments: [...prev.assignments, assignment]
      }));

      setNewAssignment({ title: '', type: 'reading', points: 100, dueDate: '' });
    }
  };

  const handleRemoveAssignment = (index) => {
    setSyllabus(prev => ({
      ...prev,
      assignments: prev.assignments.filter((_, i) => i !== index)
    }));
  };

  const handleSaveSyllabus = () => {
    // In a real app, this would be an API call
    console.log('Saving syllabus:', syllabus);
    alert('Syllabus saved successfully!');
  };

  if (loading) {
    return (
      <div className="container mx-auto px-4 py-8">
        <div className="animate-pulse">
          <div className="h-8 bg-gray-200 rounded w-1/3 mb-6"></div>
          <div className="space-y-4">
            <div className="h-4 bg-gray-200 rounded w-full"></div>
            <div className="h-4 bg-gray-200 rounded w-5/6"></div>
            <div className="h-4 bg-gray-200 rounded w-4/6"></div>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className="container mx-auto px-4 py-8">
      <div className="mb-8">
        <div className="flex justify-between items-start">
          <div>
            <h1 className="text-3xl font-bold text-gray-800">{course?.title || 'Course Syllabus'}</h1>
            <p className="text-gray-600">Instructor: {course?.instructor || 'N/A'}</p>
          </div>
          <button
            onClick={handleSaveSyllabus}
            className="bg-blue-500 hover:bg-blue-600 text-white px-6 py-2 rounded-md transition-colors"
          >
            Save Syllabus
          </button>
        </div>
      </div>

      <div className="border-b border-gray-200 mb-6">
        <nav className="-mb-px flex space-x-8">
          {[
            { id: 'overview', label: 'Overview' },
            { id: 'chapters', label: 'Chapters' },
            { id: 'assignments', label: 'Assignments' },
            { id: 'schedule', label: 'Schedule' }
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

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-8">
        <div className="lg:col-span-2">
          {activeTab === 'overview' && (
            <div className="space-y-6">
              <div className="bg-white rounded-lg shadow-md p-6">
                <h3 className="text-lg font-semibold text-gray-800 mb-4">Course Description</h3>
                <textarea
                  value={syllabus.description}
                  onChange={(e) => setSyllabus(prev => ({ ...prev, description: e.target.value }))}
                  className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500 min-h-[120px]"
                  placeholder="Enter course description..."
                />
              </div>

              <div className="bg-white rounded-lg shadow-md p-6">
                <div className="flex justify-between items-center mb-4">
                  <h3 className="text-lg font-semibold text-gray-800">Learning Objectives</h3>
                </div>
                <div className="space-y-3 mb-4">
                  {syllabus.objectives.map((objective, index) => (
                    <div key={index} className="flex items-center justify-between bg-gray-50 p-3 rounded">
                      <span className="text-gray-700">{objective}</span>
                      <button
                        onClick={() => handleRemoveObjective(index)}
                        className="text-red-600 hover:text-red-800 text-sm"
                      >
                        Remove
                      </button>
                    </div>
                  ))}
                </div>
                <div className="flex">
                  <input
                    type="text"
                    value={newObjective}
                    onChange={(e) => setNewObjective(e.target.value)}
                    onKeyPress={(e) => e.key === 'Enter' && handleAddObjective()}
                    className="flex-1 px-3 py-2 border border-gray-300 rounded-l-md focus:outline-none focus:ring-2 focus:ring-blue-500"
                    placeholder="Add a new learning objective..."
                  />
                  <button
                    onClick={handleAddObjective}
                    className="bg-blue-500 hover:bg-blue-600 text-white px-4 py-2 rounded-r-md transition-colors"
                  >
                    Add
                  </button>
                </div>
              </div>

              <div className="bg-white rounded-lg shadow-md p-6">
                <h3 className="text-lg font-semibold text-gray-800 mb-4">Grading Policy</h3>
                <textarea
                  value={syllabus.gradingPolicy}
                  onChange={(e) => setSyllabus(prev => ({ ...prev, gradingPolicy: e.target.value }))}
                  className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500 min-h-[100px]"
                  placeholder="Enter grading policy..."
                />
              </div>
            </div>
          )}

          {activeTab === 'chapters' && (
            <div className="space-y-6">
              <div className="bg-white rounded-lg shadow-md p-6">
                <div className="flex justify-between items-center mb-4">
                  <h3 className="text-lg font-semibold text-gray-800">Course Chapters</h3>
                </div>
                <div className="space-y-3 mb-4">
                  {syllabus.chapters.map((chapter, index) => (
                    <div key={index} className="flex items-center justify-between bg-gray-50 p-3 rounded">
                      <div>
                        <span className="font-medium">Ch. {chapter.order}: </span>
                        <span className="text-gray-700">{chapter.title}</span>
                        {chapter.dueDate && (
                          <span className="ml-2 text-sm text-gray-500">Due: {chapter.dueDate}</span>
                        )}
                      </div>
                      <button
                        onClick={() => handleRemoveChapter(index)}
                        className="text-red-600 hover:text-red-800 text-sm"
                      >
                        Remove
                      </button>
                    </div>
                  ))}
                </div>
                <div className="grid grid-cols-1 md:grid-cols-4 gap-2">
                  <input
                    type="text"
                    value={newChapter.title}
                    onChange={(e) => setNewChapter(prev => ({ ...prev, title: e.target.value }))}
                    className="px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
                    placeholder="Chapter title"
                  />
                  <input
                    type="number"
                    value={newChapter.order}
                    onChange={(e) => setNewChapter(prev => ({ ...prev, order: e.target.value }))}
                    className="px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
                    placeholder="Order"
                  />
                  <input
                    type="date"
                    value={newChapter.dueDate}
                    onChange={(e) => setNewChapter(prev => ({ ...prev, dueDate: e.target.value }))}
                    className="px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
                  />
                  <button
                    onClick={handleAddChapter}
                    className="bg-blue-500 hover:bg-blue-600 text-white px-4 py-2 rounded-md transition-colors"
                  >
                    Add Chapter
                  </button>
                </div>
              </div>
            </div>
          )}

          {activeTab === 'assignments' && (
            <div className="space-y-6">
              <div className="bg-white rounded-lg shadow-md p-6">
                <div className="flex justify-between items-center mb-4">
                  <h3 className="text-lg font-semibold text-gray-800">Assignments & Assessments</h3>
                </div>

                {/* Assignment Creation Form */}
                <div className="mb-6 p-4 bg-blue-50 rounded-lg">
                  <h4 className="font-medium text-gray-800 mb-3">Create New Assignment</h4>
                  <div className="grid grid-cols-1 md:grid-cols-5 gap-2">
                    <input
                      type="text"
                      value={newAssignment.title}
                      onChange={(e) => setNewAssignment(prev => ({ ...prev, title: e.target.value }))}
                      className="px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
                      placeholder="Assignment title"
                    />
                    <select
                      value={newAssignment.type}
                      onChange={(e) => setNewAssignment(prev => ({ ...prev, type: e.target.value }))}
                      className="px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
                    >
                      <option value="reading">Reading</option>
                      <option value="quiz">Quiz</option>
                      <option value="homework">Homework</option>
                      <option value="project">Project</option>
                      <option value="exam">Exam</option>
                      <option value="presentation">Presentation</option>
                      <option value="discussion">Discussion</option>
                    </select>
                    <input
                      type="number"
                      value={newAssignment.points}
                      onChange={(e) => setNewAssignment(prev => ({ ...prev, points: e.target.value }))}
                      className="px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
                      placeholder="Points"
                    />
                    <input
                      type="date"
                      value={newAssignment.dueDate}
                      onChange={(e) => setNewAssignment(prev => ({ ...prev, dueDate: e.target.value }))}
                      className="px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
                    />
                    <button
                      onClick={handleAddAssignment}
                      className="bg-blue-500 hover:bg-blue-600 text-white px-4 py-2 rounded-md transition-colors"
                    >
                      Add Assignment
                    </button>
                  </div>
                </div>

                {/* Assignments List */}
                <div className="space-y-3">
                  {syllabus.assignments.map((assignment, index) => (
                    <div key={assignment.id} className="flex items-center justify-between bg-gray-50 p-4 rounded-lg border">
                      <div className="flex-1">
                        <div className="flex items-center">
                          <span className="font-medium text-lg">{assignment.title}</span>
                          <span className={`ml-3 px-2 py-1 rounded-full text-xs ${
                            assignment.type === 'exam' ? 'bg-red-100 text-red-800' :
                            assignment.type === 'project' ? 'bg-purple-100 text-purple-800' :
                            assignment.type === 'quiz' ? 'bg-yellow-100 text-yellow-800' :
                            assignment.type === 'homework' ? 'bg-blue-100 text-blue-800' :
                            'bg-gray-100 text-gray-800'
                          }`}>
                            {assignment.type.charAt(0).toUpperCase() + assignment.type.slice(1)}
                          </span>
                        </div>
                        <div className="flex items-center mt-2 text-sm text-gray-600">
                          <span>{assignment.points} points</span>
                          {assignment.dueDate && (
                            <>
                              <span className="mx-2">•</span>
                              <span>Due: {new Date(assignment.dueDate).toLocaleDateString()}</span>
                            </>
                          )}
                        </div>
                      </div>
                      <div className="flex space-x-2">
                        <button className="text-blue-600 hover:text-blue-800 text-sm px-3 py-1">
                          Edit
                        </button>
                        <button className="text-green-600 hover:text-green-800 text-sm px-3 py-1">
                          Grade
                        </button>
                        <button
                          onClick={() => handleRemoveAssignment(index)}
                          className="text-red-600 hover:text-red-800 text-sm px-3 py-1"
                        >
                          Remove
                        </button>
                      </div>
                    </div>
                  ))}
                </div>

                {/* Assignment Statistics */}
                {syllabus.assignments.length > 0 && (
                  <div className="mt-6 pt-6 border-t border-gray-200">
                    <h4 className="font-medium text-gray-800 mb-3">Assignment Statistics</h4>
                    <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
                      <div className="text-center p-3 bg-gray-50 rounded-lg">
                        <div className="text-xl font-bold text-gray-800">{syllabus.assignments.length}</div>
                        <div className="text-sm text-gray-600">Total Assignments</div>
                      </div>
                      <div className="text-center p-3 bg-gray-50 rounded-lg">
                        <div className="text-xl font-bold text-blue-600">
                          {syllabus.assignments.reduce((sum, a) => sum + a.points, 0)}
                        </div>
                        <div className="text-sm text-gray-600">Total Points</div>
                      </div>
                      <div className="text-center p-3 bg-gray-50 rounded-lg">
                        <div className="text-xl font-bold text-green-600">
                          {Math.round(syllabus.assignments.filter(a => a.type === 'quiz' || a.type === 'exam').length / syllabus.assignments.length * 100) || 0}%
                        </div>
                        <div className="text-sm text-gray-600">Assessments</div>
                      </div>
                      <div className="text-center p-3 bg-gray-50 rounded-lg">
                        <div className="text-xl font-bold text-purple-600">
                          {Math.round(syllabus.assignments.filter(a => a.type === 'project' || a.type === 'homework').length / syllabus.assignments.length * 100) || 0}%
                        </div>
                        <div className="text-sm text-gray-600">Projects/HW</div>
                      </div>
                    </div>
                  </div>
                )}
              </div>
            </div>
          )}

          {activeTab === 'schedule' && (
            <div className="space-y-6">
              <div className="bg-white rounded-lg shadow-md p-6">
                <h3 className="text-lg font-semibold text-gray-800 mb-4">Course Schedule</h3>
                <div className="overflow-x-auto">
                  <table className="min-w-full divide-y divide-gray-200">
                    <thead className="bg-gray-50">
                      <tr>
                        <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                          Week
                        </th>
                        <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                          Topics
                        </th>
                        <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                          Readings
                        </th>
                        <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                          Assignments
                        </th>
                      </tr>
                    </thead>
                    <tbody className="bg-white divide-y divide-gray-200">
                      {syllabus.schedule.map((week, index) => (
                        <tr key={index}>
                          <td className="px-6 py-4 whitespace-nowrap text-sm font-medium text-gray-900">
                            Week {week.week}
                          </td>
                          <td className="px-6 py-4 text-sm text-gray-500">
                            <ul className="list-disc pl-5 space-y-1">
                              {week.topics.map((topic, i) => (
                                <li key={i}>{topic}</li>
                              ))}
                            </ul>
                          </td>
                          <td className="px-6 py-4 text-sm text-gray-500">
                            <ul className="list-disc pl-5 space-y-1">
                              {week.readings.map((reading, i) => (
                                <li key={i}>{reading}</li>
                              ))}
                            </ul>
                          </td>
                          <td className="px-6 py-4 text-sm text-gray-500">
                            <ul className="list-disc pl-5 space-y-1">
                              {week.assignments.map((assignment, i) => (
                                <li key={i}>{assignment}</li>
                              ))}
                            </ul>
                          </td>
                        </tr>
                      ))}
                    </tbody>
                  </table>
                </div>
              </div>
            </div>
          )}
        </div>

        <div className="lg:col-span-1">
          <div className="bg-white rounded-lg shadow-md p-6 sticky top-6">
            <h3 className="text-lg font-semibold text-gray-800 mb-4">Course Details</h3>
            <div className="space-y-3">
              <div>
                <span className="text-sm text-gray-500">Course Title</span>
                <p className="font-medium">{course?.title || 'N/A'}</p>
              </div>
              <div>
                <span className="text-sm text-gray-500">Instructor</span>
                <p className="font-medium">{course?.instructor || 'N/A'}</p>
              </div>
              <div className="grid grid-cols-2 gap-4">
                <div>
                  <span className="text-sm text-gray-500">Start Date</span>
                  <p className="font-medium">{course?.startDate || 'N/A'}</p>
                </div>
                <div>
                  <span className="text-sm text-gray-500">End Date</span>
                  <p className="font-medium">{course?.endDate || 'N/A'}</p>
                </div>
              </div>
              <div>
                <span className="text-sm text-gray-500">Credits</span>
                <p className="font-medium">{course?.credits || 'N/A'} credits</p>
              </div>
            </div>

            <div className="mt-6 pt-6 border-t border-gray-200">
              <h4 className="font-medium text-gray-800 mb-3">Quick Actions</h4>
              <div className="space-y-2">
                <button className="w-full text-left text-blue-600 hover:text-blue-800 text-sm py-1">
                  Preview Syllabus
                </button>
                <button className="w-full text-left text-blue-600 hover:text-blue-800 text-sm py-1">
                  Export to PDF
                </button>
                <button className="w-full text-left text-blue-600 hover:text-blue-800 text-sm py-1">
                  Share Syllabus
                </button>
                <button className="w-full text-left text-blue-600 hover:text-blue-800 text-sm py-1">
                  Copy to New Course
                </button>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default SyllabusPage;
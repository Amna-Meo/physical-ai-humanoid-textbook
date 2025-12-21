import React, { useState, useEffect } from 'react';
import { getApi } from '../services/api';

const InstructorDashboard = () => {
  const [activeTab, setActiveTab] = useState('overview'); // 'overview', 'courses', 'students', 'analytics'
  const [courses, setCourses] = useState([]);
  const [enrollments, setEnrollments] = useState([]);
  const [assignments, setAssignments] = useState([]);
  const [instructorAnalytics, setInstructorAnalytics] = useState(null);
  const [selectedCourseAnalytics, setSelectedCourseAnalytics] = useState(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);

  useEffect(() => {
    const fetchData = async () => {
      try {
        setLoading(true);

        // Get instructor ID from user context (in a real app, this would come from auth context)
        const instructorId = 1; // This should be replaced with actual instructor ID from auth

        // Fetch instructor analytics
        const analyticsResponse = await getApi(`/courses/instructor/${instructorId}/analytics`);
        if (analyticsResponse.ok) {
          const analyticsData = await analyticsResponse.json();
          setInstructorAnalytics(analyticsData);

          // Set courses from analytics data
          if (analyticsData.courses) {
            // Transform analytics data to match expected format
            const transformedCourses = analyticsData.courses.map(course => ({
              id: course.course_id,
              title: course.course_title,
              description: 'Course description', // Would come from course data
              studentCount: course.total_students,
              completionRate: course.completion_rate,
              active: true, // Would come from course data
              startDate: '2025-01-15', // Would come from course data
              endDate: '2025-05-20' // Would come from course data
            }));
            setCourses(transformedCourses);
          }
        }

        // Fetch detailed course data
        const coursesResponse = await getApi(`/courses/instructor/${instructorId}`);
        if (coursesResponse.ok) {
          const coursesData = await coursesResponse.json();
          // Update courses with more detailed information
          const detailedCourses = coursesData.map(course => ({
            ...course,
            studentCount: course.enrollment_count || course.total_students || 0,
            active: course.is_active
          }));
          setCourses(detailedCourses);
        }

        // For each course, fetch enrollments and assignments
        // For now, we'll use mock data for enrollments and assignments until we have the full API
        const mockEnrollments = [
          { id: 1, courseId: 1, studentName: 'John Smith', email: 'john@example.com', status: 'active', progress: 75 },
          { id: 2, courseId: 1, studentName: 'Sarah Johnson', email: 'sarah@example.com', status: 'active', progress: 45 },
          { id: 3, courseId: 1, studentName: 'Mike Chen', email: 'mike@example.com', status: 'dropped', progress: 20 },
          { id: 4, courseId: 2, studentName: 'Emma Davis', email: 'emma@example.com', status: 'active', progress: 80 },
        ];

        const mockAssignments = [
          { id: 1, courseId: 1, title: 'Chapter 1 Quiz', dueDate: '2025-01-25', submitted: 38, total: 42, avgScore: 85 },
          { id: 2, courseId: 1, title: 'Robot Simulation Project', dueDate: '2025-02-10', submitted: 15, total: 42, avgScore: 78 },
          { id: 3, courseId: 2, title: 'Midterm Exam', dueDate: '2025-03-15', submitted: 0, total: 28, avgScore: null },
        ];

        setEnrollments(mockEnrollments);
        setAssignments(mockAssignments);
      } catch (err) {
        setError(err.message);
        console.error('Error fetching dashboard data:', err);
      } finally {
        setLoading(false);
      }
    };

    fetchData();
  }, []);

  // Function to fetch course-specific analytics
  const fetchCourseAnalytics = async (courseId) => {
    try {
      const response = await getApi(`/courses/${courseId}/analytics`);
      if (response.ok) {
        const analyticsData = await response.json();
        setSelectedCourseAnalytics(analyticsData);
      }
    } catch (err) {
      console.error('Error fetching course analytics:', err);
    }
  };

  const renderOverview = () => (
    <div className="space-y-6">
      <div className="grid grid-cols-1 md:grid-cols-4 gap-6">
        <div className="bg-white rounded-lg shadow-md p-6">
          <h3 className="text-lg font-semibold text-gray-800 mb-2">Total Courses</h3>
          <p className="text-3xl font-bold text-blue-600">{courses.length}</p>
          <p className="text-sm text-gray-600 mt-1">Active: {courses.filter(c => c.active).length}</p>
        </div>

        <div className="bg-white rounded-lg shadow-md p-6">
          <h3 className="text-lg font-semibold text-gray-800 mb-2">Total Students</h3>
          <p className="text-3xl font-bold text-green-600">
            {courses.reduce((sum, course) => sum + course.studentCount, 0)}
          </p>
          <p className="text-sm text-gray-600 mt-1">Across all courses</p>
        </div>

        <div className="bg-white rounded-lg shadow-md p-6">
          <h3 className="text-lg font-semibold text-gray-800 mb-2">Avg. Completion</h3>
          <p className="text-3xl font-bold text-purple-600">
            {courses.length > 0 ? Math.round(courses.reduce((sum, course) => sum + course.completionRate, 0) / courses.length) : 0}%
          </p>
          <p className="text-sm text-gray-600 mt-1">Course completion rate</p>
        </div>

        <div className="bg-white rounded-lg shadow-md p-6">
          <h3 className="text-lg font-semibold text-gray-800 mb-2">Pending Assignments</h3>
          <p className="text-3xl font-bold text-yellow-600">
            {assignments.filter(a => new Date(a.dueDate) > new Date()).length}
          </p>
          <p className="text-sm text-gray-600 mt-1">Due in next 30 days</p>
        </div>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
        <div className="bg-white rounded-lg shadow-md p-6">
          <h3 className="text-lg font-semibold text-gray-800 mb-4">Recent Courses</h3>
          <div className="space-y-4">
            {courses.slice(0, 3).map(course => (
              <div key={course.id} className="border border-gray-200 rounded-lg p-4">
                <div className="flex justify-between items-start">
                  <div>
                    <h4 className="font-medium text-gray-800">{course.title}</h4>
                    <p className="text-sm text-gray-600">{course.description}</p>
                  </div>
                  <span className={`px-2 py-1 rounded-full text-xs ${
                    course.active ? 'bg-green-100 text-green-800' : 'bg-gray-100 text-gray-800'
                  }`}>
                    {course.active ? 'Active' : 'Inactive'}
                  </span>
                </div>
                <div className="mt-3 flex justify-between text-sm">
                  <span className="text-gray-600">{course.studentCount} students</span>
                  <span className="text-gray-600">{course.completionRate}% complete</span>
                </div>
              </div>
            ))}
          </div>
        </div>

        <div className="bg-white rounded-lg shadow-md p-6">
          <h3 className="text-lg font-semibold text-gray-800 mb-4">Upcoming Assignments</h3>
          <div className="space-y-3">
            {assignments
              .filter(a => new Date(a.dueDate) > new Date())
              .sort((a, b) => new Date(a.dueDate) - new Date(b.dueDate))
              .slice(0, 3)
              .map(assignment => (
                <div key={assignment.id} className="border border-gray-200 rounded-lg p-3">
                  <div className="flex justify-between">
                    <h4 className="font-medium text-gray-800">{assignment.title}</h4>
                    <span className="text-sm text-gray-600">{assignment.dueDate}</span>
                  </div>
                  <div className="mt-2 flex justify-between text-sm">
                    <span className="text-gray-600">{assignment.submitted}/{assignment.total} submitted</span>
                    {assignment.avgScore && (
                      <span className="text-gray-600">Avg: {assignment.avgScore}%</span>
                    )}
                  </div>
                </div>
              ))}
          </div>
        </div>
      </div>
    </div>
  );

  const renderCourses = () => (
    <div className="space-y-6">
      <div className="flex justify-between items-center">
        <h3 className="text-xl font-semibold text-gray-800">My Courses</h3>
        <button className="bg-blue-500 hover:bg-blue-600 text-white px-4 py-2 rounded-md transition-colors">
          + Create New Course
        </button>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
        {courses.map(course => (
          <div key={course.id} className="bg-white rounded-lg shadow-md overflow-hidden">
            <div className="bg-gradient-to-r from-blue-500 to-indigo-600 p-4 text-white">
              <h4 className="font-semibold">{course.title}</h4>
              <p className="text-sm opacity-80">{course.description}</p>
            </div>
            <div className="p-4">
              <div className="flex justify-between mb-3">
                <span className="text-sm text-gray-600">{course.studentCount} students</span>
                <span className={`text-xs px-2 py-1 rounded ${
                  course.active ? 'bg-green-100 text-green-800' : 'bg-gray-100 text-gray-800'
                }`}>
                  {course.active ? 'Active' : 'Inactive'}
                </span>
              </div>

              <div className="mb-3">
                <div className="flex justify-between text-sm mb-1">
                  <span className="text-gray-600">Completion</span>
                  <span className="font-medium">{course.completionRate}%</span>
                </div>
                <div className="w-full bg-gray-200 rounded-full h-2">
                  <div
                    className="bg-blue-600 h-2 rounded-full"
                    style={{ width: `${course.completionRate}%` }}
                  ></div>
                </div>
              </div>

              <div className="grid grid-cols-2 gap-2 text-xs text-gray-600">
                <div>Start: {course.startDate}</div>
                <div>End: {course.endDate}</div>
              </div>

              <div className="mt-4 flex space-x-2">
                <button className="flex-1 bg-gray-100 hover:bg-gray-200 text-gray-800 py-2 rounded text-sm transition-colors">
                  View
                </button>
                <button className="flex-1 bg-blue-100 hover:bg-blue-200 text-blue-800 py-2 rounded text-sm transition-colors">
                  Edit
                </button>
              </div>
            </div>
          </div>
        ))}
      </div>
    </div>
  );

  const renderStudents = () => (
    <div className="space-y-6">
      <h3 className="text-xl font-semibold text-gray-800">Student Roster</h3>

      <div className="bg-white rounded-lg shadow-md overflow-hidden">
        <table className="min-w-full divide-y divide-gray-200">
          <thead className="bg-gray-50">
            <tr>
              <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                Student
              </th>
              <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                Course
              </th>
              <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                Status
              </th>
              <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                Progress
              </th>
              <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">
                Actions
              </th>
            </tr>
          </thead>
          <tbody className="bg-white divide-y divide-gray-200">
            {enrollments.map(enrollment => {
              const course = courses.find(c => c.id === enrollment.courseId);
              return (
                <tr key={enrollment.id}>
                  <td className="px-6 py-4 whitespace-nowrap">
                    <div>
                      <div className="text-sm font-medium text-gray-900">{enrollment.studentName}</div>
                      <div className="text-sm text-gray-500">{enrollment.email}</div>
                    </div>
                  </td>
                  <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
                    {course?.title || 'Unknown Course'}
                  </td>
                  <td className="px-6 py-4 whitespace-nowrap">
                    <span className={`px-2 inline-flex text-xs leading-5 font-semibold rounded-full ${
                      enrollment.status === 'active' ? 'bg-green-100 text-green-800' :
                      enrollment.status === 'dropped' ? 'bg-red-100 text-red-800' :
                      'bg-yellow-100 text-yellow-800'
                    }`}>
                      {enrollment.status}
                    </span>
                  </td>
                  <td className="px-6 py-4 whitespace-nowrap">
                    <div className="flex items-center">
                      <div className="w-16 bg-gray-200 rounded-full h-2 mr-2">
                        <div
                          className="bg-blue-600 h-2 rounded-full"
                          style={{ width: `${enrollment.progress}%` }}
                        ></div>
                      </div>
                      <span className="text-sm text-gray-600">{enrollment.progress}%</span>
                    </div>
                  </td>
                  <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">
                    <button className="text-blue-600 hover:text-blue-900 mr-3">Message</button>
                    <button className="text-gray-600 hover:text-gray-900">View</button>
                  </td>
                </tr>
              );
            })}
          </tbody>
        </table>
      </div>
    </div>
  );

  const renderAnalytics = () => {
    if (!instructorAnalytics) {
      return (
        <div className="text-center py-12">
          <p className="text-gray-600">Loading analytics data...</p>
        </div>
      );
    }

    return (
      <div className="space-y-6">
        <h3 className="text-xl font-semibold text-gray-800">Course Analytics</h3>

        {/* Overall Instructor Metrics */}
        <div className="grid grid-cols-1 md:grid-cols-4 gap-6">
          <div className="bg-white rounded-lg shadow-md p-6">
            <h4 className="text-lg font-semibold text-gray-800 mb-2">Total Courses</h4>
            <p className="text-3xl font-bold text-blue-600">{instructorAnalytics.total_courses}</p>
          </div>

          <div className="bg-white rounded-lg shadow-md p-6">
            <h4 className="text-lg font-semibold text-gray-800 mb-2">Total Students</h4>
            <p className="text-3xl font-bold text-green-600">{instructorAnalytics.total_students_across_courses}</p>
          </div>

          <div className="bg-white rounded-lg shadow-md p-6">
            <h4 className="text-lg font-semibold text-gray-800 mb-2">Avg. Completion</h4>
            <p className="text-3xl font-bold text-purple-600">{instructorAnalytics.avg_course_completion_rate}%</p>
          </div>

          <div className="bg-white rounded-lg shadow-md p-6">
            <h4 className="text-lg font-semibold text-gray-800 mb-2">Active Courses</h4>
            <p className="text-3xl font-bold text-yellow-600">
              {courses.filter(c => c.active).length}
            </p>
          </div>
        </div>

        {/* Course-specific Analytics */}
        <div className="bg-white rounded-lg shadow-md p-6">
          <h4 className="font-medium text-gray-800 mb-4">Course Performance</h4>
          <div className="overflow-x-auto">
            <table className="min-w-full divide-y divide-gray-200">
              <thead className="bg-gray-50">
                <tr>
                  <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">Course</th>
                  <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">Students</th>
                  <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">Completion</th>
                  <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">Avg. Progress</th>
                  <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">Engagement</th>
                  <th className="px-6 py-3 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">Assignments</th>
                </tr>
              </thead>
              <tbody className="bg-white divide-y divide-gray-200">
                {instructorAnalytics.courses?.map(course => (
                  <tr key={course.course_id}>
                    <td className="px-6 py-4 whitespace-nowrap text-sm font-medium text-gray-900">{course.course_title}</td>
                    <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">{course.total_students}</td>
                    <td className="px-6 py-4 whitespace-nowrap">
                      <div className="text-sm text-gray-900">{course.completion_rate}%</div>
                      <div className="w-24 bg-gray-200 rounded-full h-2 mt-1">
                        <div
                          className="bg-green-600 h-2 rounded-full"
                          style={{ width: `${course.completion_rate}%` }}
                        ></div>
                      </div>
                    </td>
                    <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">{course.avg_progress}%</td>
                    <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">{course.engagement_rate}%</td>
                    <td className="px-6 py-4 whitespace-nowrap text-sm text-gray-500">{course.total_assignments}</td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        </div>

        {/* Chapter Progress Analytics */}
        <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
          <div className="bg-white rounded-lg shadow-md p-6">
            <h4 className="font-medium text-gray-800 mb-4">Chapter Progress Distribution</h4>
            <div className="space-y-4">
              {instructorAnalytics.courses?.map(course => (
                <div key={`chapters-${course.course_id}`} className="border border-gray-200 rounded-lg p-4">
                  <h5 className="font-medium text-gray-700 mb-3">{course.course_title}</h5>
                  {course.chapter_progress_stats?.slice(0, 5).map((chapter, idx) => (
                    <div key={idx} className="mb-3 last:mb-0">
                      <div className="flex justify-between text-sm mb-1">
                        <span className="text-gray-600">Chapter {chapter.order_index + 1}</span>
                        <span className="font-medium">{chapter.completion_rate}%</span>
                      </div>
                      <div className="w-full bg-gray-200 rounded-full h-2">
                        <div
                          className="bg-blue-600 h-2 rounded-full"
                          style={{ width: `${chapter.completion_rate}%` }}
                        ></div>
                      </div>
                      <div className="text-xs text-gray-500 mt-1">
                        {chapter.students_completed}/{chapter.total_students} students completed
                      </div>
                    </div>
                  ))}
                  {(!course.chapter_progress_stats || course.chapter_progress_stats.length === 0) && (
                    <p className="text-gray-500 text-sm">No chapter progress data available</p>
                  )}
                </div>
              ))}
            </div>
          </div>

          {/* Assignment Analytics */}
          <div className="bg-white rounded-lg shadow-md p-6">
            <h4 className="font-medium text-gray-800 mb-4">Assignment Performance</h4>
            <div className="space-y-4">
              {instructorAnalytics.courses?.map(course => (
                <div key={`assignments-${course.course_id}`} className="border border-gray-200 rounded-lg p-4">
                  <h5 className="font-medium text-gray-700 mb-3">{course.course_title}</h5>
                  <div className="space-y-2">
                    <div className="flex justify-between text-sm">
                      <span className="text-gray-600">Total Assignments:</span>
                      <span className="font-medium">{course.total_assignments}</span>
                    </div>
                    <div className="flex justify-between text-sm">
                      <span className="text-gray-600">Completion Rate:</span>
                      <span className="font-medium">{course.assignment_completion_rate}%</span>
                    </div>
                    <div className="flex justify-between text-sm">
                      <span className="text-gray-600">Avg. Score:</span>
                      <span className="font-medium">{course.avg_assignment_score ? course.avg_assignment_score + '%' : 'N/A'}</span>
                    </div>
                    <div className="w-full bg-gray-200 rounded-full h-2 mt-2">
                      <div
                        className="bg-purple-600 h-2 rounded-full"
                        style={{ width: `${course.assignment_completion_rate}%` }}
                      ></div>
                    </div>
                  </div>
                </div>
              ))}
            </div>
          </div>
        </div>

        {/* Engagement Metrics */}
        <div className="bg-white rounded-lg shadow-md p-6">
          <h4 className="font-medium text-gray-800 mb-4">Student Engagement Overview</h4>
          <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
            <div className="text-center p-4 bg-blue-50 rounded-lg">
              <div className="text-2xl font-bold text-blue-600">
                {instructorAnalytics.courses ?
                  Math.round(instructorAnalytics.courses.reduce((sum, c) => sum + c.engagement_rate, 0) / instructorAnalytics.courses.length) || 0
                  : 0}%
              </div>
              <div className="text-sm text-gray-600">Avg. Engagement Rate</div>
            </div>
            <div className="text-center p-4 bg-green-50 rounded-lg">
              <div className="text-2xl font-bold text-green-600">
                {instructorAnalytics.courses ?
                  Math.round(instructorAnalytics.courses.reduce((sum, c) => sum + c.avg_progress, 0) / instructorAnalytics.courses.length) || 0
                  : 0}%
              </div>
              <div className="text-sm text-gray-600">Avg. Progress</div>
            </div>
            <div className="text-center p-4 bg-purple-50 rounded-lg">
              <div className="text-2xl font-bold text-purple-600">
                {instructorAnalytics.courses ?
                  instructorAnalytics.courses.reduce((sum, c) => sum + c.recent_activity, 0)
                  : 0}
              </div>
              <div className="text-sm text-gray-600">Recent Activity</div>
            </div>
          </div>
        </div>
      </div>
    );
  };

  if (loading) {
    return (
      <div className="container mx-auto px-4 py-8">
        <div className="animate-pulse">
          <div className="h-8 bg-gray-200 rounded w-1/4 mb-6"></div>
          <div className="space-y-4">
            <div className="h-4 bg-gray-200 rounded w-3/4"></div>
            <div className="h-4 bg-gray-200 rounded w-1/2"></div>
            <div className="h-4 bg-gray-200 rounded w-2/3"></div>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className="container mx-auto px-4 py-8">
      <div className="mb-8">
        <h1 className="text-3xl font-bold text-gray-800">Instructor Dashboard</h1>
        <p className="text-gray-600">Manage your courses, students, and content</p>
      </div>

      <div className="border-b border-gray-200 mb-6">
        <nav className="-mb-px flex space-x-8">
          {[
            { id: 'overview', label: 'Overview' },
            { id: 'courses', label: 'My Courses' },
            { id: 'students', label: 'Students' },
            { id: 'analytics', label: 'Analytics' }
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
        {activeTab === 'courses' && renderCourses()}
        {activeTab === 'students' && renderStudents()}
        {activeTab === 'analytics' && renderAnalytics()}
      </div>
    </div>
  );
};

export default InstructorDashboard;
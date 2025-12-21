import React, { useState, useEffect } from 'react';
import { useParams } from 'react-router-dom';
import { apiService } from '../services/apiService';
import { UserLearningPaths, PersonalizedLearningPathGenerator } from '../components/LearningPath';
import BackgroundAssessmentQuestionnaire from '../components/BackgroundAssessmentQuestionnaire';

const ProfessionalDevelopment = () => {
  const { userId } = useParams();
  const [user, setUser] = useState(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);
  const [activeTab, setActiveTab] = useState('dashboard');
  const [recommendations, setRecommendations] = useState({ content: [], learningPaths: [] });
  const [showAssessment, setShowAssessment] = useState(false);

  useEffect(() => {
    loadUserData();
  }, [userId]);

  useEffect(() => {
    if (user) {
      loadRecommendations();
    }
  }, [user]);

  const loadUserData = async () => {
    try {
      setLoading(true);
      setError(null);

      const userData = await apiService.getMe();
      setUser(userData);
    } catch (err) {
      setError('Failed to load user data: ' + err.message);
      console.error('Error loading user data:', err);
    } finally {
      setLoading(false);
    }
  };

  const loadRecommendations = async () => {
    try {
      // Load content recommendations
      const contentRecs = await apiService.getContentRecommendations(userId);
      // Load learning path recommendations
      const pathRecs = await apiService.getLearningPathRecommendations(userId);

      setRecommendations({
        content: contentRecs.recommendations || [],
        learningPaths: pathRecs.recommendations || []
      });
    } catch (err) {
      console.error('Error loading recommendations:', err);
      // Don't show error for recommendations as they're supplementary
    }
  };

  const handleAssessmentComplete = (answers) => {
    setShowAssessment(false);
    // Reload user data to reflect updated profile
    loadUserData();
  };

  const handleStartLearningPath = async (learningPathId) => {
    try {
      await apiService.assignLearningPathToUser(userId, learningPathId);
      // Refresh the page or update state as needed
      window.location.reload();
    } catch (err) {
      setError('Failed to start learning path: ' + err.message);
      console.error('Error starting learning path:', err);
    }
  };

  if (loading) {
    return (
      <div className="professional-development-page">
        <div className="loading">Loading professional development dashboard...</div>
      </div>
    );
  }

  if (error) {
    return (
      <div className="professional-development-page">
        <div className="error">Error: {error}</div>
      </div>
    );
  }

  return (
    <div className="professional-development-page">
      <div className="page-header">
        <h1>Professional Development Dashboard</h1>
        <p>Personalized learning paths and content based on your professional background</p>
      </div>

      {showAssessment ? (
        <div className="assessment-overlay">
          <div className="assessment-container">
            <h2>Professional Background Assessment</h2>
            <p>Complete this assessment to get personalized recommendations</p>
            <BackgroundAssessmentQuestionnaire
              userId={userId}
              onComplete={handleAssessmentComplete}
            />
            <button
              className="btn btn-secondary"
              onClick={() => setShowAssessment(false)}
            >
              Close Assessment
            </button>
          </div>
        </div>
      ) : null}

      <div className="dashboard-tabs">
        <button
          className={`tab-button ${activeTab === 'dashboard' ? 'active' : ''}`}
          onClick={() => setActiveTab('dashboard')}
        >
          Dashboard
        </button>
        <button
          className={`tab-button ${activeTab === 'learning-paths' ? 'active' : ''}`}
          onClick={() => setActiveTab('learning-paths')}
        >
          Learning Paths
        </button>
        <button
          className={`tab-button ${activeTab === 'recommendations' ? 'active' : ''}`}
          onClick={() => setActiveTab('recommendations')}
        >
          Recommendations
        </button>
        <button
          className={`tab-button ${activeTab === 'profile' ? 'active' : ''}`}
          onClick={() => setActiveTab('profile')}
        >
          Profile & Preferences
        </button>
      </div>

      <div className="dashboard-content">
        {activeTab === 'dashboard' && (
          <div className="dashboard-overview">
            <div className="user-summary">
              <h2>Welcome, {user?.full_name || user?.username}!</h2>
              <p>Based on your profile, we've created personalized learning recommendations.</p>

              {!user?.professional_background && (
                <div className="profile-completion-notice">
                  <p>✨ Complete your professional background assessment to get better recommendations!</p>
                  <button
                    className="btn btn-primary"
                    onClick={() => setShowAssessment(true)}
                  >
                    Complete Assessment
                  </button>
                </div>
              )}
            </div>

            <div className="quick-stats">
              <div className="stat-card">
                <h3>Your Learning Paths</h3>
                <p>Track your progress</p>
              </div>
              <div className="stat-card">
                <h3>Recommended Content</h3>
                <p>Personalized for you</p>
              </div>
              <div className="stat-card">
                <h3>Skills Gained</h3>
                <p>Track your growth</p>
              </div>
            </div>

            <div className="recent-activity">
              <h3>Recent Activity</h3>
              <p>No recent activity yet. Start a learning path to begin!</p>
            </div>
          </div>
        )}

        {activeTab === 'learning-paths' && (
          <div className="learning-paths-section">
            <div className="section-header">
              <h2>Your Learning Paths</h2>
              <PersonalizedLearningPathGenerator userId={userId} />
            </div>

            <UserLearningPaths userId={userId} />

            <div className="available-paths">
              <h3>Available Learning Paths</h3>
              <p>Explore our collection of learning paths</p>
              {/* This would be populated with available learning paths */}
            </div>
          </div>
        )}

        {activeTab === 'recommendations' && (
          <div className="recommendations-section">
            <div className="section-header">
              <h2>Personalized Recommendations</h2>
              <p>Content and learning paths tailored to your background and goals</p>
            </div>

            <div className="content-recommendations">
              <h3>Recommended Content</h3>
              {recommendations.content.length > 0 ? (
                <div className="recommendations-grid">
                  {recommendations.content.map((item, index) => (
                    <div key={index} className="recommendation-card">
                      <h4>{item.title}</h4>
                      <p>{item.description}</p>
                      <div className="recommendation-meta">
                        <span className="relevance-score">Relevance: {Math.round(item.relevance_score * 10) / 10}</span>
                        <span className="reading-time">{item.reading_time_minutes} min read</span>
                      </div>
                      <div className="tags">
                        {item.tags.map((tag, tagIndex) => (
                          <span key={tagIndex} className="tag">{tag}</span>
                        ))}
                      </div>
                      <button className="btn btn-outline">Start Reading</button>
                    </div>
                  ))}
                </div>
              ) : (
                <p>No content recommendations available. Complete your profile assessment for better recommendations.</p>
              )}
            </div>

            <div className="path-recommendations">
              <h3>Recommended Learning Paths</h3>
              {recommendations.learningPaths.length > 0 ? (
                <div className="recommendations-grid">
                  {recommendations.learningPaths.map((item, index) => (
                    <div key={index} className="recommendation-card">
                      <h4>{item.title}</h4>
                      <p>{item.description}</p>
                      <div className="recommendation-meta">
                        <span className="difficulty">Difficulty: {item.difficulty_level}</span>
                        <span className="duration">~{item.estimated_duration_hours} hours</span>
                        <span className="relevance-score">Match: {Math.round(item.relevance_score * 10) / 10}</span>
                      </div>
                      <button
                        className="btn btn-primary"
                        onClick={() => handleStartLearningPath(item.id)}
                      >
                        Start Learning Path
                      </button>
                    </div>
                  ))}
                </div>
              ) : (
                <p>No learning path recommendations available. Complete your profile assessment for better recommendations.</p>
              )}
            </div>
          </div>
        )}

        {activeTab === 'profile' && (
          <div className="profile-section">
            <div className="section-header">
              <h2>Profile & Preferences</h2>
              <button
                className="btn btn-primary"
                onClick={() => setShowAssessment(true)}
              >
                Update Assessment
              </button>
            </div>

            <div className="profile-details">
              <h3>Professional Background</h3>
              <p><strong>Current Role:</strong> {user?.professional_background || 'Not specified'}</p>

              <h3>Learning Preferences</h3>
              <p><strong>Preferred Pace:</strong> {user?.learning_preferences?.preferred_pace || 'Not specified'}</p>
              <p><strong>Primary Goal:</strong> {user?.learning_preferences?.primary_goal || 'Not specified'}</p>
              <p><strong>Time Availability:</strong> {user?.learning_preferences?.time_availability || 'Not specified'}</p>

              <h3>Technical Skills</h3>
              <p><strong>Programming Languages:</strong> {user?.programming_languages?.join(', ') || 'Not specified'}</p>
            </div>
          </div>
        )}
      </div>
    </div>
  );
};

export default ProfessionalDevelopment;
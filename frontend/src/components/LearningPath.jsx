import React, { useState, useEffect } from 'react';
import { apiService } from '../services/apiService';

const LearningPath = ({ userId, learningPathId }) => {
  const [learningPath, setLearningPath] = useState(null);
  const [steps, setSteps] = useState([]);
  const [userProgress, setUserProgress] = useState({});
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);

  useEffect(() => {
    if (userId && learningPathId) {
      loadLearningPathData();
    }
  }, [userId, learningPathId]);

  const loadLearningPathData = async () => {
    try {
      setLoading(true);
      setError(null);

      // Fetch learning path details
      const pathResponse = await apiService.getLearningPath(learningPathId);
      setLearningPath(pathResponse);

      // Fetch learning path steps
      const stepsResponse = await apiService.getLearningPathSteps(learningPathId);
      setSteps(stepsResponse);

      // Fetch user progress for this learning path
      const progressResponse = await apiService.getUserLearningPathProgress(userId, learningPathId);
      if (progressResponse) {
        setUserProgress(progressResponse);
      }
    } catch (err) {
      setError('Failed to load learning path data: ' + err.message);
      console.error('Error loading learning path:', err);
    } finally {
      setLoading(false);
    }
  };

  const updateStepProgress = async (stepId, status) => {
    try {
      const response = await apiService.updateLearningPathStepProgress(userId, learningPathId, stepId, status);

      // Update local state
      loadLearningPathData(); // Reload to get updated progress
    } catch (err) {
      setError('Failed to update step progress: ' + err.message);
      console.error('Error updating step progress:', err);
    }
  };

  const markStepComplete = (stepId) => {
    updateStepProgress(stepId, 'completed');
  };

  const markStepInProgress = (stepId) => {
    updateStepProgress(stepId, 'in_progress');
  };

  if (loading) {
    return (
      <div className="learning-path-container">
        <div className="loading">Loading learning path...</div>
      </div>
    );
  }

  if (error) {
    return (
      <div className="learning-path-container">
        <div className="error">Error: {error}</div>
      </div>
    );
  }

  if (!learningPath) {
    return (
      <div className="learning-path-container">
        <div className="no-data">No learning path found.</div>
      </div>
    );
  }

  return (
    <div className="learning-path-container">
      <div className="learning-path-header">
        <h2>{learningPath.title}</h2>
        <p className="learning-path-description">{learningPath.description}</p>
        <div className="learning-path-meta">
          <span className="difficulty">Difficulty: {learningPath.difficulty_level || 'Unknown'}</span>
          <span className="duration">Duration: ~{learningPath.estimated_duration_hours || '0'} hours</span>
          {userProgress && (
            <span className="progress">Progress: {userProgress.progress_percentage || 0}%</span>
          )}
        </div>
      </div>

      <div className="learning-path-steps">
        <h3>Learning Steps</h3>
        {steps.length === 0 ? (
          <p>No steps defined for this learning path.</p>
        ) : (
          <div className="steps-list">
            {steps.map((step, index) => {
              const userStepProgress = null; // In a real implementation, we'd fetch this data
              const isCompleted = userStepProgress?.status === 'completed';
              const isInProgress = userStepProgress?.status === 'in_progress';

              return (
                <div key={step.id} className={`step-item ${isCompleted ? 'completed' : isInProgress ? 'in-progress' : ''}`}>
                  <div className="step-header">
                    <div className="step-number">Step {step.step_number}</div>
                    <div className="step-title">{step.title}</div>
                    <div className="step-duration">{step.estimated_duration_minutes || 0} min</div>
                  </div>

                  <div className="step-content">
                    <p className="step-description">{step.description}</p>
                    <div className="step-type">Type: {step.content_type}</div>
                    <div className="step-actions">
                      {!isCompleted ? (
                        <>
                          <button
                            className="btn btn-primary"
                            onClick={() => markStepInProgress(step.id)}
                          >
                            Start
                          </button>
                          <button
                            className="btn btn-success"
                            onClick={() => markStepComplete(step.id)}
                          >
                            Mark Complete
                          </button>
                        </>
                      ) : (
                        <button
                          className="btn btn-secondary"
                          onClick={() => updateStepProgress(step.id, 'not_started')}
                        >
                          Mark Incomplete
                        </button>
                      )}
                    </div>
                  </div>
                </div>
              );
            })}
          </div>
        )}
      </div>
    </div>
  );
};

// Component for displaying user's learning paths
const UserLearningPaths = ({ userId }) => {
  const [userLearningPaths, setUserLearningPaths] = useState([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);

  useEffect(() => {
    if (userId) {
      loadUserLearningPaths();
    }
  }, [userId]);

  const loadUserLearningPaths = async () => {
    try {
      setLoading(true);
      setError(null);

      const response = await apiService.getUserLearningPaths(userId);
      setUserLearningPaths(response);
    } catch (err) {
      setError('Failed to load user learning paths: ' + err.message);
      console.error('Error loading user learning paths:', err);
    } finally {
      setLoading(false);
    }
  };

  if (loading) {
    return <div>Loading your learning paths...</div>;
  }

  if (error) {
    return <div>Error: {error}</div>;
  }

  return (
    <div className="user-learning-paths">
      <h2>Your Learning Paths</h2>
      {userLearningPaths.length === 0 ? (
        <p>You don't have any learning paths assigned yet.</p>
      ) : (
        <div className="learning-paths-grid">
          {userLearningPaths.map((userPath) => (
            <div key={userPath.id} className="learning-path-card">
              <h3>{userPath.learning_path.title}</h3>
              <p>{userPath.learning_path.description}</p>
              <div className="path-progress">
                <span>Progress: {userPath.progress_percentage}%</span>
                <span>Status: {userPath.status}</span>
              </div>
              <button
                className="btn btn-primary"
                onClick={() => window.location.href = `/learning-path/${userPath.learning_path_id}`}
              >
                Continue Learning
              </button>
            </div>
          ))}
        </div>
      )}
    </div>
  );
};

// Component for generating personalized learning paths
const PersonalizedLearningPathGenerator = ({ userId }) => {
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);
  const [success, setSuccess] = useState(null);

  const generatePersonalizedPath = async () => {
    try {
      setLoading(true);
      setError(null);
      setSuccess(null);

      const response = await apiService.generatePersonalizedLearningPath(userId);
      setSuccess('Personalized learning path generated successfully!');
      console.log('Generated path:', response);
    } catch (err) {
      setError('Failed to generate personalized learning path: ' + err.message);
      console.error('Error generating personalized path:', err);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="personalized-path-generator">
      <h3>Generate Personalized Learning Path</h3>
      <p>Based on your professional background and learning preferences, we can create a personalized learning path for you.</p>
      <button
        className="btn btn-primary"
        onClick={generatePersonalizedPath}
        disabled={loading}
      >
        {loading ? 'Generating...' : 'Generate Personalized Path'}
      </button>
      {success && <div className="success">{success}</div>}
      {error && <div className="error">{error}</div>}
    </div>
  );
};

export { LearningPath, UserLearningPaths, PersonalizedLearningPathGenerator };
export default LearningPath;
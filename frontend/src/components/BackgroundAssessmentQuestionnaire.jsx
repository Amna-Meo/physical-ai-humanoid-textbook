import React, { useState, useEffect } from 'react';
import { apiService } from '../services/apiService';

const BackgroundAssessmentQuestionnaire = ({ userId, onComplete }) => {
  const [currentStep, setCurrentStep] = useState(0);
  const [answers, setAnswers] = useState({
    // Professional background
    professional_background: '',
    current_role: '',
    years_experience: '',
    technical_skills: [],
    programming_languages: [],

    // Educational background
    education_level: '',
    field_of_study: '',
    academic_experience: '',

    // Learning preferences
    learning_preferences: {
      preferred_pace: 'moderate',
      content_format: [],
      learning_style: '',
      time_availability: '',
      primary_goal: ''
    },

    // Domain interests
    robotics_interests: [],
    ai_interests: [],
    specific_topics: ''
  });

  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);

  const questionnaireSteps = [
    {
      id: 'professional',
      title: 'Professional Background',
      description: 'Tell us about your professional experience',
      fields: [
        {
          name: 'current_role',
          label: 'Current Role/Position',
          type: 'text',
          placeholder: 'e.g., Software Engineer, Robotics Researcher, Student'
        },
        {
          name: 'years_experience',
          label: 'Years of Experience',
          type: 'select',
          options: [
            { value: '0-1', label: '0-1 years' },
            { value: '1-3', label: '1-3 years' },
            { value: '3-5', label: '3-5 years' },
            { value: '5-10', label: '5-10 years' },
            { value: '10+', label: '10+ years' }
          ]
        },
        {
          name: 'professional_background',
          label: 'Professional Background',
          type: 'textarea',
          placeholder: 'Describe your professional background, previous roles, and areas of expertise...'
        }
      ]
    },
    {
      id: 'education',
      title: 'Educational Background',
      description: 'Tell us about your educational experience',
      fields: [
        {
          name: 'education_level',
          label: 'Education Level',
          type: 'select',
          options: [
            { value: 'high_school', label: 'High School' },
            { value: 'bachelor', label: 'Bachelor\'s Degree' },
            { value: 'master', label: 'Master\'s Degree' },
            { value: 'phd', label: 'PhD' },
            { value: 'other', label: 'Other' }
          ]
        },
        {
          name: 'field_of_study',
          label: 'Field of Study',
          type: 'text',
          placeholder: 'e.g., Computer Science, Robotics, Engineering, Mathematics'
        },
        {
          name: 'academic_experience',
          label: 'Academic Experience',
          type: 'textarea',
          placeholder: 'Describe your academic background, research experience, publications, etc...'
        }
      ]
    },
    {
      id: 'skills',
      title: 'Technical Skills',
      description: 'What technical skills do you have?',
      fields: [
        {
          name: 'technical_skills',
          label: 'Technical Skills',
          type: 'multiselect',
          options: [
            { value: 'robotics', label: 'Robotics' },
            { value: 'machine_learning', label: 'Machine Learning' },
            { value: 'computer_vision', label: 'Computer Vision' },
            { value: 'control_systems', label: 'Control Systems' },
            { value: 'simulation', label: 'Simulation' },
            { value: 'embedded_systems', label: 'Embedded Systems' },
            { value: 'real_time_systems', label: 'Real-time Systems' },
            { value: 'ai_planning', label: 'AI Planning' },
            { value: 'human_robot_interaction', label: 'Human-Robot Interaction' }
          ]
        },
        {
          name: 'programming_languages',
          label: 'Programming Languages',
          type: 'multiselect',
          options: [
            { value: 'python', label: 'Python' },
            { value: 'c++', label: 'C++' },
            { value: 'c', label: 'C' },
            { value: 'java', label: 'Java' },
            { value: 'javascript', label: 'JavaScript' },
            { value: 'matlab', label: 'MATLAB' },
            { value: 'ros', label: 'ROS/ROS2' },
            { value: 'other', label: 'Other' }
          ]
        }
      ]
    },
    {
      id: 'preferences',
      title: 'Learning Preferences',
      description: 'How do you prefer to learn?',
      fields: [
        {
          name: 'preferred_pace',
          label: 'Preferred Learning Pace',
          type: 'select',
          options: [
            { value: 'slow', label: 'Slow and thorough' },
            { value: 'moderate', label: 'Moderate pace' },
            { value: 'fast', label: 'Fast-paced, challenging' }
          ]
        },
        {
          name: 'time_availability',
          label: 'Time Availability',
          type: 'select',
          options: [
            { value: 'less_2', label: 'Less than 2 hours per week' },
            { value: '2_5', label: '2-5 hours per week' },
            { value: '5_10', label: '5-10 hours per week' },
            { value: '10_20', label: '10-20 hours per week' },
            { value: '20_plus', label: '20+ hours per week' }
          ]
        },
        {
          name: 'content_format',
          label: 'Preferred Content Format',
          type: 'multiselect',
          options: [
            { value: 'text', label: 'Text-based content' },
            { value: 'video', label: 'Video lectures' },
            { value: 'interactive', label: 'Interactive simulations' },
            { value: 'hands_on', label: 'Hands-on exercises' },
            { value: 'theoretical', label: 'Theoretical concepts' },
            { value: 'practical', label: 'Practical applications' }
          ]
        },
        {
          name: 'primary_goal',
          label: 'Primary Goal',
          type: 'select',
          options: [
            { value: 'career_advancement', label: 'Career advancement' },
            { value: 'academic_pursuit', label: 'Academic pursuit' },
            { value: 'personal_interest', label: 'Personal interest' },
            { value: 'project_development', label: 'Project development' },
            { value: 'research', label: 'Research' }
          ]
        }
      ]
    },
    {
      id: 'interests',
      title: 'Areas of Interest',
      description: 'What topics are you most interested in?',
      fields: [
        {
          name: 'robotics_interests',
          label: 'Robotics Interests',
          type: 'multiselect',
          options: [
            { value: 'humanoid_robots', label: 'Humanoid Robots' },
            { value: 'mobile_robots', label: 'Mobile Robots' },
            { value: 'manipulation', label: 'Manipulation' },
            { value: 'locomotion', label: 'Locomotion' },
            { value: 'swarm_robots', label: 'Swarm Robotics' },
            { value: 'field_robots', label: 'Field Robotics' },
            { value: 'service_robots', label: 'Service Robots' }
          ]
        },
        {
          name: 'ai_interests',
          label: 'AI Interests',
          type: 'multiselect',
          options: [
            { value: 'deep_learning', label: 'Deep Learning' },
            { value: 'reinforcement_learning', label: 'Reinforcement Learning' },
            { value: 'computer_vision', label: 'Computer Vision' },
            { value: 'nlp', label: 'Natural Language Processing' },
            { value: 'planning', label: 'AI Planning' },
            { value: 'multi_agent', label: 'Multi-agent Systems' },
            { value: 'cognitive_arch', label: 'Cognitive Architectures' }
          ]
        },
        {
          name: 'specific_topics',
          label: 'Specific Topics of Interest',
          type: 'textarea',
          placeholder: 'Any specific topics, problems, or applications you\'re interested in...'
        }
      ]
    }
  ];

  const currentStepData = questionnaireSteps[currentStep];

  const handleInputChange = (fieldName, value) => {
    setAnswers(prev => ({
      ...prev,
      [fieldName]: value
    }));
  };

  const handleNestedInputChange = (section, fieldName, value) => {
    setAnswers(prev => ({
      ...prev,
      [section]: {
        ...prev[section],
        [fieldName]: value
      }
    }));
  };

  const handleArrayChange = (fieldName, value, checked) => {
    setAnswers(prev => {
      const currentArray = Array.isArray(prev[fieldName]) ? prev[fieldName] : [];
      let newArray;

      if (checked) {
        newArray = [...currentArray, value];
      } else {
        newArray = currentArray.filter(item => item !== value);
      }

      return {
        ...prev,
        [fieldName]: newArray
      };
    });
  };

  const handleNestedArrayChange = (section, fieldName, value, checked) => {
    setAnswers(prev => {
      const currentArray = Array.isArray(prev[section][fieldName]) ? prev[section][fieldName] : [];
      let newArray;

      if (checked) {
        newArray = [...currentArray, value];
      } else {
        newArray = currentArray.filter(item => item !== value);
      }

      return {
        ...prev,
        [section]: {
          ...prev[section],
          [fieldName]: newArray
        }
      };
    });
  };

  const nextStep = () => {
    if (currentStep < questionnaireSteps.length - 1) {
      setCurrentStep(currentStep + 1);
    }
  };

  const prevStep = () => {
    if (currentStep > 0) {
      setCurrentStep(currentStep - 1);
    }
  };

  const handleSubmit = async () => {
    try {
      setLoading(true);
      setError(null);

      // Prepare the data to update user profile
      const profileUpdate = {
        professional_background: answers.professional_background,
        learning_preferences: JSON.stringify(answers.learning_preferences)
      };

      // Update user profile with the assessment data
      await apiService.updateProfile(profileUpdate);

      // Optionally generate a personalized learning path based on the assessment
      try {
        await apiService.generatePersonalizedLearningPath(userId);
      } catch (pathError) {
        console.warn('Could not generate personalized learning path:', pathError.message);
      }

      if (onComplete) {
        onComplete(answers);
      }
    } catch (err) {
      setError('Failed to save your responses: ' + err.message);
      console.error('Error saving questionnaire:', err);
    } finally {
      setLoading(false);
    }
  };

  const renderField = (field) => {
    const value = field.name.includes('.')
      ? answers[field.name.split('.')[0]][field.name.split('.')[1]]
      : answers[field.name];

    switch (field.type) {
      case 'text':
      case 'textarea':
        return (
          <div key={field.name} className="form-group">
            <label htmlFor={field.name}>{field.label}</label>
            {field.type === 'textarea' ? (
              <textarea
                id={field.name}
                value={value || ''}
                onChange={(e) => handleInputChange(field.name, e.target.value)}
                placeholder={field.placeholder}
                className="form-control"
              />
            ) : (
              <input
                type="text"
                id={field.name}
                value={value || ''}
                onChange={(e) => handleInputChange(field.name, e.target.value)}
                placeholder={field.placeholder}
                className="form-control"
              />
            )}
          </div>
        );

      case 'select':
        return (
          <div key={field.name} className="form-group">
            <label htmlFor={field.name}>{field.label}</label>
            <select
              id={field.name}
              value={value || ''}
              onChange={(e) => handleInputChange(field.name, e.target.value)}
              className="form-control"
            >
              <option value="">Select an option</option>
              {field.options.map(option => (
                <option key={option.value} value={option.value}>
                  {option.label}
                </option>
              ))}
            </select>
          </div>
        );

      case 'multiselect':
        return (
          <div key={field.name} className="form-group">
            <label>{field.label}</label>
            <div className="checkbox-group">
              {field.options.map(option => (
                <div key={option.value} className="checkbox-item">
                  <input
                    type="checkbox"
                    id={`${field.name}-${option.value}`}
                    checked={(value || []).includes(option.value)}
                    onChange={(e) => handleArrayChange(field.name, option.value, e.target.checked)}
                  />
                  <label htmlFor={`${field.name}-${option.value}`}>{option.label}</label>
                </div>
              ))}
            </div>
          </div>
        );

      default:
        return null;
    }
  };

  return (
    <div className="background-assessment-questionnaire">
      <div className="questionnaire-header">
        <h2>Professional Background Assessment</h2>
        <p className="questionnaire-description">
          Help us understand your background and preferences to create a personalized learning experience.
        </p>
        <div className="progress-bar">
          <div
            className="progress-fill"
            style={{ width: `${((currentStep + 1) / questionnaireSteps.length) * 100}%` }}
          ></div>
          <span className="progress-text">
            Step {currentStep + 1} of {questionnaireSteps.length}
          </span>
        </div>
      </div>

      <div className="questionnaire-form">
        <h3>{currentStepData.title}</h3>
        <p className="step-description">{currentStepData.description}</p>

        {error && <div className="error-message">{error}</div>}

        {currentStepData.fields.map(field => renderField(field))}
      </div>

      <div className="questionnaire-navigation">
        <button
          className="btn btn-secondary"
          onClick={prevStep}
          disabled={currentStep === 0}
        >
          Previous
        </button>

        {currentStep < questionnaireSteps.length - 1 ? (
          <button
            className="btn btn-primary"
            onClick={nextStep}
          >
            Next
          </button>
        ) : (
          <button
            className="btn btn-success"
            onClick={handleSubmit}
            disabled={loading}
          >
            {loading ? 'Saving...' : 'Complete Assessment'}
          </button>
        )}
      </div>

      <div className="questionnaire-steps-indicator">
        {questionnaireSteps.map((step, index) => (
          <div
            key={step.id}
            className={`step-indicator ${index === currentStep ? 'active' : index < currentStep ? 'completed' : ''}`}
            onClick={() => setCurrentStep(index)}
          >
            <span className="step-number">{index + 1}</span>
            <span className="step-label">{step.title}</span>
          </div>
        ))}
      </div>
    </div>
  );
};

export default BackgroundAssessmentQuestionnaire;
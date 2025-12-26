// API Service for Physical AI & Humanoid Robotics Textbook

const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || 'http://39.34.129.27:8000/api/v1';

// Default headers
const defaultHeaders = {
  'Content-Type': 'application/json',
};

// Utility function to get auth token from localStorage
const getAuthToken = () => {
  return localStorage.getItem('access_token');
};

// Utility function to add auth header
const getAuthHeaders = () => {
  const token = getAuthToken();
  return token
    ? { ...defaultHeaders, Authorization: `Bearer ${token}` }
    : defaultHeaders;
};

// API service class
class ApiService {
  // Authentication methods
  async login(username, password) {
    const response = await fetch(`${API_BASE_URL}/users/token`, {
      method: 'POST',
      headers: defaultHeaders,
      body: JSON.stringify({
        username,
        password,
      }),
    });

    if (!response.ok) {
      throw new Error(`Login failed: ${response.statusText}`);
    }

    const data = await response.json();
    // Store token in localStorage
    if (data.access_token) {
      localStorage.setItem('access_token', data.access_token);
    }

    return data;
  }

  async register(userData) {
    const response = await fetch(`${API_BASE_URL}/users/register`, {
      method: 'POST',
      headers: defaultHeaders,
      body: JSON.stringify(userData),
    });

    if (!response.ok) {
      throw new Error(`Registration failed: ${response.statusText}`);
    }

    return await response.json();
  }

  async logout() {
    // Remove token from localStorage
    localStorage.removeItem('access_token');
  }

  async getMe() {
    const response = await fetch(`${API_BASE_URL}/users/me`, {
      method: 'GET',
      headers: getAuthHeaders(),
    });

    if (!response.ok) {
      throw new Error(`Failed to get user: ${response.statusText}`);
    }

    return await response.json();
  }

  async updateProfile(userData) {
    const response = await fetch(`${API_BASE_URL}/users/me`, {
      method: 'PUT',
      headers: getAuthHeaders(),
      body: JSON.stringify(userData),
    });

    if (!response.ok) {
      throw new Error(`Failed to update profile: ${response.statusText}`);
    }

    return await response.json();
  }

  // Chapter methods
  async getChapters(filters = {}) {
    const params = new URLSearchParams(filters);
    const queryString = params.toString();
    const url = queryString ? `${API_BASE_URL}/chapters?${queryString}` : `${API_BASE_URL}/chapters`;

    const response = await fetch(url, {
      method: 'GET',
      headers: getAuthHeaders(),
    });

    if (!response.ok) {
      throw new Error(`Failed to get chapters: ${response.statusText}`);
    }

    return await response.json();
  }

  async getChapterById(id) {
    const response = await fetch(`${API_BASE_URL}/chapters/${id}`, {
      method: 'GET',
      headers: getAuthHeaders(),
    });

    if (!response.ok) {
      throw new Error(`Failed to get chapter: ${response.statusText}`);
    }

    return await response.json();
  }

  // Learning path methods
  async getLearningPaths() {
    const response = await fetch(`${API_BASE_URL}/learning-paths`, {
      method: 'GET',
      headers: getAuthHeaders(),
    });

    if (!response.ok) {
      throw new Error(`Failed to get learning paths: ${response.statusText}`);
    }

    return await response.json();
  }

  async getLearningPathById(id) {
    const response = await fetch(`${API_BASE_URL}/learning-paths/${id}`, {
      method: 'GET',
      headers: getAuthHeaders(),
    });

    if (!response.ok) {
      throw new Error(`Failed to get learning path: ${response.statusText}`);
    }

    return await response.json();
  }

  async createLearningPath(learningPathData) {
    const response = await fetch(`${API_BASE_URL}/learning-paths`, {
      method: 'POST',
      headers: getAuthHeaders(),
      body: JSON.stringify(learningPathData),
    });

    if (!response.ok) {
      throw new Error(`Failed to create learning path: ${response.statusText}`);
    }

    return await response.json();
  }

  async updateLearningPath(id, learningPathData) {
    const response = await fetch(`${API_BASE_URL}/learning-paths/${id}`, {
      method: 'PUT',
      headers: getAuthHeaders(),
      body: JSON.stringify(learningPathData),
    });

    if (!response.ok) {
      throw new Error(`Failed to update learning path: ${response.statusText}`);
    }

    return await response.json();
  }

  async deleteLearningPath(id) {
    const response = await fetch(`${API_BASE_URL}/learning-paths/${id}`, {
      method: 'DELETE',
      headers: getAuthHeaders(),
    });

    if (!response.ok) {
      throw new Error(`Failed to delete learning path: ${response.statusText}`);
    }

    return await response.json();
  }

  async getLearningPathSteps(learningPathId) {
    const response = await fetch(`${API_BASE_URL}/learning-paths/${learningPathId}/steps`, {
      method: 'GET',
      headers: getAuthHeaders(),
    });

    if (!response.ok) {
      throw new Error(`Failed to get learning path steps: ${response.statusText}`);
    }

    return await response.json();
  }

  async getUserLearningPaths(userId) {
    const response = await fetch(`${API_BASE_URL}/learning-paths/user/${userId}`, {
      method: 'GET',
      headers: getAuthHeaders(),
    });

    if (!response.ok) {
      throw new Error(`Failed to get user learning paths: ${response.statusText}`);
    }

    return await response.json();
  }

  async assignLearningPathToUser(userId, learningPathId) {
    const response = await fetch(`${API_BASE_URL}/learning-paths/assign/${userId}/${learningPathId}`, {
      method: 'POST',
      headers: getAuthHeaders(),
    });

    if (!response.ok) {
      throw new Error(`Failed to assign learning path to user: ${response.statusText}`);
    }

    return await response.json();
  }

  async generatePersonalizedLearningPath(userId) {
    const response = await fetch(`${API_BASE_URL}/learning-paths/generate-personalized/${userId}`, {
      method: 'POST',
      headers: getAuthHeaders(),
    });

    if (!response.ok) {
      throw new Error(`Failed to generate personalized learning path: ${response.statusText}`);
    }

    return await response.json();
  }

  async updateLearningPathStepProgress(userId, learningPathId, stepId, status) {
    const response = await fetch(`${API_BASE_URL}/user-learning-path-progress`, {
      method: 'POST',
      headers: getAuthHeaders(),
      body: JSON.stringify({
        user_id: userId,
        learning_path_id: learningPathId,
        step_id: stepId,
        status: status
      }),
    });

    if (!response.ok) {
      throw new Error(`Failed to update learning path step progress: ${response.statusText}`);
    }

    return await response.json();
  }

  async getUserLearningPathProgress(userId, learningPathId) {
    const response = await fetch(`${API_BASE_URL}/learning-paths/user/${userId}/progress/${learningPathId}`, {
      method: 'GET',
      headers: getAuthHeaders(),
    });

    if (!response.ok) {
      throw new Error(`Failed to get user learning path progress: ${response.statusText}`);
    }

    return await response.json();
  }

  // AI interaction methods
  async createChatSession(title = null) {
    const response = await fetch(`${API_BASE_URL}/ai/chat-sessions`, {
      method: 'POST',
      headers: getAuthHeaders(),
      body: JSON.stringify({ session_title: title }),
    });

    if (!response.ok) {
      throw new Error(`Failed to create chat session: ${response.statusText}`);
    }

    return await response.json();
  }

  async getChatSession(id) {
    const response = await fetch(`${API_BASE_URL}/ai/chat-sessions/${id}`, {
      method: 'GET',
      headers: getAuthHeaders(),
    });

    if (!response.ok) {
      throw new Error(`Failed to get chat session: ${response.statusText}`);
    }

    return await response.json();
  }

  async sendMessage(sessionId, message) {
    const response = await fetch(`${API_BASE_URL}/ai/chat-sessions/${sessionId}/messages`, {
      method: 'POST',
      headers: getAuthHeaders(),
      body: JSON.stringify({ content: message, role: 'user', message_type: 'user' }),
    });

    if (!response.ok) {
      throw new Error(`Failed to send message: ${response.statusText}`);
    }

    return await response.json();
  }

  async searchContent(query, filters = {}) {
    const params = new URLSearchParams({ query, ...filters });
    const response = await fetch(`${API_BASE_URL}/ai/search?${params.toString()}`, {
      method: 'GET',
      headers: getAuthHeaders(),
    });

    if (!response.ok) {
      throw new Error(`Failed to search content: ${response.statusText}`);
    }

    return await response.json();
  }

  // Utility methods
  isAuthenticated() {
    return !!getAuthToken();
  }

  setAuthToken(token) {
    localStorage.setItem('access_token', token);
  }

  removeAuthToken() {
    localStorage.removeItem('access_token');
  }
}

// Generic API function for making requests
export const getApi = async (endpoint) => {
  const token = getAuthToken();
  const headers = token
    ? { ...defaultHeaders, Authorization: `Bearer ${token}` }
    : defaultHeaders;

  return fetch(`${API_BASE_URL}${endpoint}`, {
    method: 'GET',
    headers: headers,
  });
};

export const postApi = async (endpoint, data) => {
  const token = getAuthToken();
  const headers = token
    ? { ...defaultHeaders, Authorization: `Bearer ${token}` }
    : defaultHeaders;

  return fetch(`${API_BASE_URL}${endpoint}`, {
    method: 'POST',
    headers: headers,
    body: JSON.stringify(data),
  });
};

export const putApi = async (endpoint, data) => {
  const token = getAuthToken();
  const headers = token
    ? { ...defaultHeaders, Authorization: `Bearer ${token}` }
    : defaultHeaders;

  return fetch(`${API_BASE_URL}${endpoint}`, {
    method: 'PUT',
    headers: headers,
    body: JSON.stringify(data),
  });
};

export const deleteApi = async (endpoint) => {
  const token = getAuthToken();
  const headers = token
    ? { ...defaultHeaders, Authorization: `Bearer ${token}` }
    : defaultHeaders;

  return fetch(`${API_BASE_URL}${endpoint}`, {
    method: 'DELETE',
    headers: headers,
  });
};

// Create and export a single instance of the API service
const apiService = new ApiService();
export default apiService;

// Export for use in React hooks
export const useApi = () => {
  return apiService;
};
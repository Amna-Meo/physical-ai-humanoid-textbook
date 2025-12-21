import React, { useState, useEffect } from 'react';

const UserProfile = () => {
  const [user, setUser] = useState(null);
  const [loading, setLoading] = useState(true);
  const [editing, setEditing] = useState(false);
  const [formData, setFormData] = useState({
    full_name: '',
    professional_background: '',
    learning_preferences: '',
    timezone: '',
    language_preference: 'en'
  });

  // Simulate fetching user data
  useEffect(() => {
    // In a real app, this would be an API call
    const fetchUser = async () => {
      try {
        // Simulate API call delay
        await new Promise(resolve => setTimeout(resolve, 500));

        // Mock user data
        const mockUser = {
          id: 1,
          email: 'student@example.com',
          username: 'student123',
          full_name: 'John Doe',
          professional_background: 'Software Engineer with 5 years experience in robotics',
          learning_preferences: '{"difficulty_level": "intermediate", "explanation_style": "detailed", "notification_frequency": "daily"}',
          timezone: 'UTC-5',
          language_preference: 'en',
          created_at: '2025-01-01T00:00:00Z',
          updated_at: '2025-01-02T00:00:00Z'
        };

        setUser(mockUser);
        setFormData({
          full_name: mockUser.full_name || '',
          professional_background: mockUser.professional_background || '',
          learning_preferences: mockUser.learning_preferences || '',
          timezone: mockUser.timezone || '',
          language_preference: mockUser.language_preference || 'en'
        });
        setLoading(false);
      } catch (error) {
        console.error('Error fetching user:', error);
        setLoading(false);
      }
    };

    fetchUser();
  }, []);

  const handleInputChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleSave = async (e) => {
    e.preventDefault();
    try {
      // In a real app, this would be an API call to update user
      console.log('Saving user profile:', formData);

      // Simulate API call
      await new Promise(resolve => setTimeout(resolve, 500));

      // Update local user state
      setUser(prev => ({
        ...prev,
        ...formData,
        updated_at: new Date().toISOString()
      }));

      setEditing(false);
      alert('Profile updated successfully!');
    } catch (error) {
      console.error('Error updating profile:', error);
      alert('Error updating profile');
    }
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

  if (!user) {
    return (
      <div className="container mx-auto px-4 py-8">
        <div className="text-center">
          <h2 className="text-2xl font-bold">User Profile</h2>
          <p className="text-gray-600 mt-4">User not found</p>
        </div>
      </div>
    );
  }

  return (
    <div className="container mx-auto px-4 py-8 max-w-3xl">
      <div className="bg-white rounded-lg shadow-md p-6">
        <div className="flex justify-between items-center mb-6">
          <h2 className="text-2xl font-bold text-gray-800">User Profile</h2>
          {!editing && (
            <button
              onClick={() => setEditing(true)}
              className="bg-blue-500 hover:bg-blue-600 text-white px-4 py-2 rounded-md transition-colors"
            >
              Edit Profile
            </button>
          )}
        </div>

        {editing ? (
          <form onSubmit={handleSave}>
            <div className="space-y-4">
              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  Full Name
                </label>
                <input
                  type="text"
                  name="full_name"
                  value={formData.full_name}
                  onChange={handleInputChange}
                  className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
                  placeholder="Enter your full name"
                />
              </div>

              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  Professional Background
                </label>
                <textarea
                  name="professional_background"
                  value={formData.professional_background}
                  onChange={handleInputChange}
                  rows="3"
                  className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
                  placeholder="Describe your professional background"
                />
              </div>

              <div>
                <label className="block text-sm font-medium text-gray-700 mb-1">
                  Learning Preferences (JSON)
                </label>
                <textarea
                  name="learning_preferences"
                  value={formData.learning_preferences}
                  onChange={handleInputChange}
                  rows="4"
                  className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500 font-mono text-sm"
                  placeholder='{"difficulty_level": "intermediate", "explanation_style": "detailed"}'
                />
              </div>

              <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                <div>
                  <label className="block text-sm font-medium text-gray-700 mb-1">
                    Timezone
                  </label>
                  <input
                    type="text"
                    name="timezone"
                    value={formData.timezone}
                    onChange={handleInputChange}
                    className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
                    placeholder="e.g., UTC-5"
                  />
                </div>

                <div>
                  <label className="block text-sm font-medium text-gray-700 mb-1">
                    Language Preference
                  </label>
                  <select
                    name="language_preference"
                    value={formData.language_preference}
                    onChange={handleInputChange}
                    className="w-full px-3 py-2 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-blue-500"
                  >
                    <option value="en">English</option>
                    <option value="es">Spanish</option>
                    <option value="fr">French</option>
                    <option value="de">German</option>
                    <option value="ur">Urdu</option>
                  </select>
                </div>
              </div>
            </div>

            <div className="flex space-x-3 mt-6">
              <button
                type="submit"
                className="bg-green-500 hover:bg-green-600 text-white px-4 py-2 rounded-md transition-colors"
              >
                Save Changes
              </button>
              <button
                type="button"
                onClick={() => {
                  setEditing(false);
                  // Reset form to original values
                  setFormData({
                    full_name: user.full_name || '',
                    professional_background: user.professional_background || '',
                    learning_preferences: user.learning_preferences || '',
                    timezone: user.timezone || '',
                    language_preference: user.language_preference || 'en'
                  });
                }}
                className="bg-gray-500 hover:bg-gray-600 text-white px-4 py-2 rounded-md transition-colors"
              >
                Cancel
              </button>
            </div>
          </form>
        ) : (
          <div className="space-y-4">
            <div>
              <h3 className="text-sm font-medium text-gray-500">Email</h3>
              <p className="text-gray-800">{user.email}</p>
            </div>

            <div>
              <h3 className="text-sm font-medium text-gray-500">Username</h3>
              <p className="text-gray-800">{user.username}</p>
            </div>

            <div>
              <h3 className="text-sm font-medium text-gray-500">Full Name</h3>
              <p className="text-gray-800">{user.full_name || 'Not provided'}</p>
            </div>

            <div>
              <h3 className="text-sm font-medium text-gray-500">Professional Background</h3>
              <p className="text-gray-800">{user.professional_background || 'Not provided'}</p>
            </div>

            <div>
              <h3 className="text-sm font-medium text-gray-500">Learning Preferences</h3>
              <pre className="text-gray-800 bg-gray-50 p-3 rounded-md font-mono text-sm overflow-x-auto">
                {user.learning_preferences || '{}'}
              </pre>
            </div>

            <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
              <div>
                <h3 className="text-sm font-medium text-gray-500">Timezone</h3>
                <p className="text-gray-800">{user.timezone || 'Not provided'}</p>
              </div>

              <div>
                <h3 className="text-sm font-medium text-gray-500">Language Preference</h3>
                <p className="text-gray-800">{user.language_preference || 'en'}</p>
              </div>
            </div>

            <div className="pt-4 border-t border-gray-200">
              <h3 className="text-sm font-medium text-gray-500">Account Information</h3>
              <div className="grid grid-cols-1 md:grid-cols-2 gap-2 mt-1">
                <div>
                  <span className="text-gray-600">Member since: </span>
                  <span className="text-gray-800">{new Date(user.created_at).toLocaleDateString()}</span>
                </div>
                <div>
                  <span className="text-gray-600">Last updated: </span>
                  <span className="text-gray-800">{user.updated_at ? new Date(user.updated_at).toLocaleDateString() : 'Never'}</span>
                </div>
              </div>
            </div>
          </div>
        )}
      </div>
    </div>
  );
};

export default UserProfile;
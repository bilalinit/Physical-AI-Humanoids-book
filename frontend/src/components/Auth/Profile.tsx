/**
 * Profile component for viewing user profile (read-only)
 */
import React from 'react';
import { useAuth } from '../../hooks/useAuth';

const Profile: React.FC = () => {
  const { user, loading, error } = useAuth();

  if (loading) {
    return <div>Loading profile...</div>;
  }

  if (!user) {
    return <div>Please sign in to view your profile.</div>;
  }

  if (error) {
    return <div>Error loading profile: {error}</div>;
  }

  return (
    <div className="profile-container">
      <h2>User Profile</h2>

      <div className="profile-display">
        <div className="profile-info" style={{ lineHeight: '2' }}>
          <div className="profile-field">
            <strong>Name:</strong> {user.name}
          </div>
          <div className="profile-field">
            <strong>Email:</strong> {user.email}
          </div>
          <div className="profile-field">
            <strong>Education Level:</strong> {user.educationLevel || 'Not specified'}
          </div>
          <div className="profile-field">
            <strong>Programming Experience:</strong> {user.programmingExperience || 'Not specified'}
          </div>
          <div className="profile-field">
            <strong>Robotics Background:</strong> {user.roboticsBackground || 'Not specified'}
          </div>
          <div className="profile-field">
            <strong>Member Since:</strong> {user.createdAt ? new Date(user.createdAt).toLocaleDateString() : 'N/A'}
          </div>
        </div>
      </div>
    </div>
  );
};

export default Profile;
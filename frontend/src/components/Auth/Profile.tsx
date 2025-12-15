/**
 * Profile component for viewing user profile (read-only)
 */
import React from 'react';
import { useAuth } from '../../hooks/useAuth';
import styles from './Profile.module.css';

const Profile: React.FC = () => {
  const { user, loading, error } = useAuth();

  if (loading) {
    return (
      <div className={styles.container}>
        <div className={styles.card}>
          <h2 className={styles.title}>User Profile</h2>
          <div className={styles.loading}>
            <div className={styles.spinner}></div>
          </div>
        </div>
      </div>
    );
  }

  if (!user) {
    return (
      <div className={styles.container}>
        <div className={styles.card}>
          <h2 className={styles.title}>User Profile</h2>
          <div>Please sign in to view your profile.</div>
        </div>
      </div>
    );
  }

  if (error) {
    return (
      <div className={styles.container}>
        <div className={styles.card}>
          <h2 className={styles.title}>User Profile</h2>
          <div className={styles.error}>Error loading profile: {error}</div>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.container}>
      <div className={styles.card}>
        <h2 className={styles.title}>User Profile</h2>

        <div className={styles.profileInfo}>
          <div className={styles.field}>
            <span className={styles.label}>Name:</span>
            <span className={styles.value}>{user.name}</span>
          </div>
          <div className={styles.field}>
            <span className={styles.label}>Email:</span>
            <span className={styles.value}>{user.email}</span>
          </div>
          <div className={styles.field}>
            <span className={styles.label}>Education Level:</span>
            <span className={styles.value}>{user.educationLevel || 'Not specified'}</span>
          </div>
          <div className={styles.field}>
            <span className={styles.label}>Programming Experience:</span>
            <span className={styles.value}>{user.programmingExperience || 'Not specified'}</span>
          </div>
          <div className={styles.field}>
            <span className={styles.label}>Robotics Background:</span>
            <span className={styles.value}>{user.roboticsBackground || 'Not specified'}</span>
          </div>
          <div className={styles.field}>
            <span className={styles.label}>Member Since:</span>
            <span className={styles.value}>{user.createdAt ? new Date(user.createdAt).toLocaleDateString() : 'N/A'}</span>
          </div>
        </div>
      </div>
    </div>
  );
};

export default Profile;
import React, { useState } from 'react';
import { usePersonalization } from '../../contexts/PersonalizationContext';
import { useAuth } from '../../contexts/AuthContext';
import styles from './PersonalizationPanel.module.css';

export default function PersonalizationPanel(): JSX.Element {
  const { isAuthenticated, user } = useAuth();
  const {
    experienceLevel,
    setExperienceLevel,
    isPersonalizationEnabled,
    togglePersonalization,
    learningGoals
  } = usePersonalization();

  const [isOpen, setIsOpen] = useState(false);

  const levels = [
    { value: 'beginner', label: 'Beginner', icon: '🌱', description: 'New to robotics and AI' },
    { value: 'intermediate', label: 'Intermediate', icon: '🚀', description: 'Some programming experience' },
    { value: 'advanced', label: 'Advanced', icon: '⚡', description: 'Strong technical background' },
    { value: 'all', label: 'Show All', icon: '🌐', description: 'View all content levels' }
  ];

  if (!isAuthenticated) {
    return null; // Don't show for unauthenticated users
  }

  return (
    <div className={styles.container}>
      <button
        className={styles.toggleButton}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Open personalization settings"
        title="Content Personalization"
      >
        <span className={styles.toggleIcon}>⚙️</span>
        <span className={styles.toggleLabel}>Content Level</span>
      </button>

      {isOpen && (
        <div className={styles.panel}>
          <div className={styles.header}>
            <h3 className={styles.title}>Content Personalization</h3>
            <button
              className={styles.closeButton}
              onClick={() => setIsOpen(false)}
              aria-label="Close"
            >
              ✕
            </button>
          </div>

          <div className={styles.content}>
            {/* Personalization Toggle */}
            <div className={styles.section}>
              <label className={styles.switchLabel}>
                <input
                  type="checkbox"
                  checked={isPersonalizationEnabled}
                  onChange={togglePersonalization}
                  className={styles.checkbox}
                />
                <span className={styles.switchText}>
                  Enable personalized content filtering
                </span>
              </label>
              <p className={styles.description}>
                {isPersonalizationEnabled
                  ? 'Content is filtered based on your experience level'
                  : 'All content is visible regardless of difficulty'}
              </p>
            </div>

            {/* Experience Level Selection */}
            {isPersonalizationEnabled && (
              <div className={styles.section}>
                <h4 className={styles.sectionTitle}>Your Experience Level</h4>
                <div className={styles.levelOptions}>
                  {levels.map((level) => (
                    <button
                      key={level.value}
                      className={`${styles.levelButton} ${
                        experienceLevel === level.value ? styles.active : ''
                      }`}
                      onClick={() => setExperienceLevel(level.value as any)}
                    >
                      <span className={styles.levelIcon}>{level.icon}</span>
                      <div className={styles.levelInfo}>
                        <span className={styles.levelLabel}>{level.label}</span>
                        <span className={styles.levelDescription}>{level.description}</span>
                      </div>
                      {experienceLevel === level.value && (
                        <span className={styles.checkmark}>✓</span>
                      )}
                    </button>
                  ))}
                </div>
              </div>
            )}

            {/* Learning Goals Summary */}
            {learningGoals && learningGoals.length > 0 && (
              <div className={styles.section}>
                <h4 className={styles.sectionTitle}>Your Learning Goals</h4>
                <div className={styles.goals}>
                  {learningGoals.map((goal, index) => (
                    <span key={index} className={styles.goalTag}>
                      {goal}
                    </span>
                  ))}
                </div>
              </div>
            )}

            {/* Current Profile Info */}
            <div className={styles.section}>
              <div className={styles.profileInfo}>
                <p className={styles.infoText}>
                  <strong>Current Level:</strong>{' '}
                  {levels.find(l => l.value === experienceLevel)?.label || 'Not set'}
                </p>
                {user?.hardware_profile?.robotics_kit && (
                  <p className={styles.infoText}>
                    <strong>Hardware:</strong> {user.hardware_profile.robotics_kit}
                  </p>
                )}
              </div>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}

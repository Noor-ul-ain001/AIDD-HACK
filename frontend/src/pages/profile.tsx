import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../contexts/AuthContext';
import { AuthAPI, UserProfile } from '../lib/api';
import clsx from 'clsx';
import styles from './profile.module.css';

type FormState = {
  email: string;
  hardwareProfile: {
    gpu?: string;
    roboticsKit?: string;
    experienceLevel?: string;
  };
  learningGoals: string[];
  accessibilityPrefs: Record<string, any>;
};

export default function ProfilePage(): JSX.Element {
  const { user, refreshUser } = useAuth();
  const [formData, setFormData] = useState<FormState>({
    email: '',
    hardwareProfile: {},
    learningGoals: [],
    accessibilityPrefs: {}
  });
  const [isEditing, setIsEditing] = useState(false);
  const [isSaving, setIsSaving] = useState(false);
  const [successMessage, setSuccessMessage] = useState('');
  const [error, setError] = useState('');

  // Initialize form data when user data is available
  useEffect(() => {
    if (user) {
      setFormData({
        email: user.email,
        hardwareProfile: user.hardware_profile || {},
        learningGoals: user.learning_goals || [],
        accessibilityPrefs: user.accessibility_prefs || {}
      });
    }
  }, [user]);

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement | HTMLTextAreaElement | HTMLSelectElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleHardwareProfileChange = (field: string, value: string) => {
    setFormData(prev => ({
      ...prev,
      hardwareProfile: {
        ...prev.hardwareProfile,
        [field]: value
      }
    }));
  };

  const handleLearningGoalChange = (goal: string) => {
    setFormData(prev => {
      const newGoals = prev.learningGoals.includes(goal)
        ? prev.learningGoals.filter(g => g !== goal)
        : [...prev.learningGoals, goal];
      
      return {
        ...prev,
        learningGoals: newGoals
      };
    });
  };

  const handleAccessibilityPrefChange = (pref: string, value: any) => {
    setFormData(prev => ({
      ...prev,
      accessibilityPrefs: {
        ...prev.accessibilityPrefs,
        [pref]: value
      }
    }));
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setIsSaving(true);
    setError('');
    setSuccessMessage('');

    try {
      const result = await AuthAPI.updateProfile({
        email: formData.email,
        hardware_profile: formData.hardwareProfile,
        learning_goals: formData.learningGoals,
        accessibility_prefs: formData.accessibilityPrefs
      });

      if (result.error) {
        setError(result.error);
      } else {
        await refreshUser(); // Refresh user data after successful update
        setSuccessMessage('Profile updated successfully!');
        setIsEditing(false);
      }
    } catch (err) {
      setError('Failed to update profile. Please try again.');
    } finally {
      setIsSaving(false);
    }
  };

  const resendVerificationEmail = async () => {
    try {
      const result = await AuthAPI.resendVerificationEmail();

      if (result.error) {
        setError(result.error);
        setSuccessMessage('');
      } else {
        setSuccessMessage('Verification email sent successfully! Please check your inbox.');
        setError('');
      }
    } catch (err) {
      setError('Failed to resend verification email. Please try again.');
      setSuccessMessage('');
    }
  };

  const learningGoalsOptions = [
    'ROS 2 Development',
    'Simulation Environments',
    'NVIDIA Isaac Sim',
    'VLA Models',
    'Hardware Integration',
    'AI for Robotics',
    'Computer Vision',
    'Motion Planning'
  ];

  if (!user) {
    return (
      <Layout title="Profile" description="User profile page">
        <main className={styles.main}>
          <div className={styles.container}>
            <div className={styles.loadingState}>
              <p>Please log in to view your profile.</p>
            </div>
          </div>
        </main>
      </Layout>
    );
  }

  return (
    <Layout title="Profile" description="View and edit your profile">
      <main className={styles.main}>
        <div className={styles.container}>
          <div className={styles.profileHeader}>
            <h1>My Profile</h1>
            <p>Manage your account information and preferences</p>
          </div>

          {successMessage && (
            <div className={styles.successMessage}>
              {successMessage}
            </div>
          )}

          {error && (
            <div className={styles.errorMessage}>
              {error}
            </div>
          )}

          <div className={styles.profileCard}>
            <div className={styles.profileInfo}>
              <h2>Personal Information</h2>

              {user && !user.email_verified && (
                <div className={styles.emailVerificationBanner}>
                  <div className={styles.bannerContent}>
                    <span className={styles.bannerIcon}>✉️</span>
                    <div className={styles.bannerText}>
                      <p><strong>Email Not Verified:</strong> Your email address has not been verified yet. Please check your inbox for the verification email.</p>
                      <p className={styles.resendVerification}>Need another verification email? <button type="button" className={styles.resendButton} onClick={resendVerificationEmail}>Resend Verification</button></p>
                    </div>
                  </div>
                </div>
              )}

              {user && user.email_verified && (
                <div className={styles.emailVerifiedBanner}>
                  <div className={styles.bannerContent}>
                    <span className={styles.bannerIcon}>✅</span>
                    <div className={styles.bannerText}>
                      <p><strong>Email Verified:</strong> Your email address has been verified.</p>
                    </div>
                  </div>
                </div>
              )}

              <form onSubmit={handleSubmit} className={styles.profileForm}>
                <div className={styles.formGroup}>
                  <label htmlFor="email" className={styles.label}>Email Address</label>
                  <input
                    type="email"
                    id="email"
                    name="email"
                    value={formData.email}
                    onChange={handleInputChange}
                    disabled={!isEditing}
                    className={clsx(styles.input, !isEditing && styles.disabled)}
                  />
                </div>

                <div className={styles.formGroup}>
                  <label className={styles.label}>Hardware Profile</label>
                  
                  <div className={styles.hardwareOptions}>
                    <div className={styles.optionGroup}>
                      <label className={styles.optionLabel}>GPU</label>
                      <select
                        value={formData.hardwareProfile.gpu || ''}
                        onChange={(e) => handleHardwareProfileChange('gpu', e.target.value)}
                        disabled={!isEditing}
                        className={clsx(styles.select, !isEditing && styles.disabled)}
                      >
                        <option value="">Select your GPU</option>
                        <option value="none">No dedicated GPU</option>
                        <option value="gtx1660">GTX 1660</option>
                        <option value="rtx2060">RTX 2060</option>
                        <option value="rtx3070">RTX 3070</option>
                        <option value="rtx4080">RTX 4080</option>
                        <option value="other">Other</option>
                      </select>
                    </div>

                    <div className={styles.optionGroup}>
                      <label className={styles.optionLabel}>Robotics Kit</label>
                      <select
                        value={formData.hardwareProfile.roboticsKit || ''}
                        onChange={(e) => handleHardwareProfileChange('roboticsKit', e.target.value)}
                        disabled={!isEditing}
                        className={clsx(styles.select, !isEditing && styles.disabled)}
                      >
                        <option value="">Select your kit</option>
                        <option value="none">No hardware</option>
                        <option value="jetson">Jetson Orin</option>
                        <option value="unitree_go1">Unitree Go1</option>
                        <option value="unitree_g1">Unitree G1</option>
                        <option value="other">Other</option>
                      </select>
                    </div>

                    <div className={styles.optionGroup}>
                      <label className={styles.optionLabel}>Experience Level</label>
                      <select
                        value={formData.hardwareProfile.experienceLevel || ''}
                        onChange={(e) => handleHardwareProfileChange('experienceLevel', e.target.value)}
                        disabled={!isEditing}
                        className={clsx(styles.select, !isEditing && styles.disabled)}
                      >
                        <option value="">Select level</option>
                        <option value="beginner">Beginner</option>
                        <option value="intermediate">Intermediate</option>
                        <option value="advanced">Advanced</option>
                      </select>
                    </div>
                  </div>
                </div>

                <div className={styles.formGroup}>
                  <label className={styles.label}>Learning Goals</label>
                  <div className={styles.checkboxGroup}>
                    {learningGoalsOptions.map((goal) => (
                      <label key={goal} className={styles.checkboxLabel}>
                        <input
                          type="checkbox"
                          checked={formData.learningGoals.includes(goal)}
                          onChange={() => handleLearningGoalChange(goal)}
                          disabled={!isEditing}
                          className={styles.checkbox}
                        />
                        <span className={styles.checkboxText}>{goal}</span>
                      </label>
                    ))}
                  </div>
                </div>

                <div className={styles.formGroup}>
                  <label className={styles.label}>Accessibility Preferences</label>
                  <div className={styles.checkboxGroup}>
                    <label className={styles.checkboxLabel}>
                      <input
                        type="checkbox"
                        checked={!!formData.accessibilityPrefs?.highContrast}
                        onChange={(e) => handleAccessibilityPrefChange('highContrast', e.target.checked)}
                        disabled={!isEditing}
                        className={styles.checkbox}
                      />
                      <span className={styles.checkboxText}>High contrast mode</span>
                    </label>
                    <label className={styles.checkboxLabel}>
                      <input
                        type="checkbox"
                        checked={!!formData.accessibilityPrefs?.largeText}
                        onChange={(e) => handleAccessibilityPrefChange('largeText', e.target.checked)}
                        disabled={!isEditing}
                        className={styles.checkbox}
                      />
                      <span className={styles.checkboxText}>Larger text size</span>
                    </label>
                    <label className={styles.checkboxLabel}>
                      <input
                        type="checkbox"
                        checked={!!formData.accessibilityPrefs?.reduceAnimations}
                        onChange={(e) => handleAccessibilityPrefChange('reduceAnimations', e.target.checked)}
                        disabled={!isEditing}
                        className={styles.checkbox}
                      />
                      <span className={styles.checkboxText}>Reduce animations</span>
                    </label>
                  </div>
                </div>

                <div className={styles.buttonGroup}>
                  {!isEditing ? (
                    <button
                      type="button"
                      onClick={() => setIsEditing(true)}
                      className={styles.editButton}
                    >
                      Edit Profile
                    </button>
                  ) : (
                    <>
                      <button
                        type="submit"
                        disabled={isSaving}
                        className={styles.saveButton}
                      >
                        {isSaving ? 'Saving...' : 'Save Changes'}
                      </button>
                      <button
                        type="button"
                        onClick={() => {
                          setIsEditing(false);
                          // Reset form to original values
                          if (user) {
                            setFormData({
                              email: user.email,
                              hardwareProfile: user.hardware_profile || {},
                              learningGoals: user.learning_goals || [],
                              accessibilityPrefs: user.accessibility_prefs || {}
                            });
                          }
                        }}
                        className={styles.cancelButton}
                      >
                        Cancel
                      </button>
                    </>
                  )}
                </div>
              </form>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}
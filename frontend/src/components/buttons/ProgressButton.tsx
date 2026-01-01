import React, { useState, useEffect } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import { ProgressAPI } from '../../lib/api';
import styles from './ProgressButton.module.css';

interface ProgressButtonProps {
  chapterId: string;
  completionPercentage?: number; // 0-100, defaults to 100
  onProgressUpdated?: (success: boolean) => void;
}

export default function ProgressButton({
  chapterId,
  completionPercentage = 100,
  onProgressUpdated
}: ProgressButtonProps): JSX.Element {
  const { isAuthenticated } = useAuth();
  const [isUpdating, setIsUpdating] = useState(false);
  const [isCompleted, setIsCompleted] = useState(false);
  const [showSuccess, setShowSuccess] = useState(false);
  const [showError, setShowError] = useState(false);
  const [errorMessage, setErrorMessage] = useState('');

  // Check if this chapter is already completed
  useEffect(() => {
    const checkProgress = async () => {
      if (!isAuthenticated) return;

      try {
        const response = await ProgressAPI.getAll();
        if (response.data) {
          const chapterProgress = response.data.find(
            (p) => p.chapter_id === chapterId
          );
          if (chapterProgress && chapterProgress.completion_percentage >= 100) {
            setIsCompleted(true);
          }
        }
      } catch (error) {
        console.error('Error checking progress:', error);
      }
    };

    checkProgress();
  }, [chapterId, isAuthenticated]);

  const handleMarkComplete = async () => {
    if (!isAuthenticated) {
      setErrorMessage('Please sign in to track your progress');
      setShowError(true);
      setTimeout(() => setShowError(false), 3000);
      return;
    }

    setIsUpdating(true);
    setShowError(false);

    console.log('[ProgressButton] Marking chapter as complete:', chapterId);

    try {
      const response = await ProgressAPI.update(
        chapterId,
        completionPercentage,
        0, // time_spent_seconds - can be tracked separately
        [], // exercises_completed - can be added later
        {} // quiz_scores - can be added later
      );

      console.log('[ProgressButton] API Response:', response);

      if (response.error) {
        throw new Error(response.error);
      }

      setIsCompleted(true);
      setShowSuccess(true);
      setTimeout(() => setShowSuccess(false), 3000);

      console.log('[ProgressButton] Progress updated successfully');

      // Notify parent component
      if (onProgressUpdated) {
        onProgressUpdated(true);
      }
    } catch (error) {
      console.error('[ProgressButton] Error updating progress:', error);
      setErrorMessage(error instanceof Error ? error.message : 'Failed to update progress');
      setShowError(true);
      setTimeout(() => setShowError(false), 3000);

      // Notify parent component
      if (onProgressUpdated) {
        onProgressUpdated(false);
      }
    } finally {
      setIsUpdating(false);
    }
  };

  if (!isAuthenticated) {
    return (
      <div className={styles.progressButton} style={{ opacity: 0.5, cursor: 'not-allowed' }}>
        <span className={styles.icon}>🔒</span>
        <span className={styles.text}>Sign in to track progress</span>
      </div>
    );
  }

  return (
    <div>
      <button
        className={`${styles.progressButton} ${isCompleted ? styles.completed : ''}`}
        onClick={handleMarkComplete}
        disabled={isUpdating || isCompleted}
      >
        {isUpdating ? (
          <>
            <span className={`${styles.icon} ${styles.loading}`}>⏳</span>
            <span className={styles.text}>Updating...</span>
          </>
        ) : isCompleted ? (
          <>
            <span className={styles.icon}>✅</span>
            <span className={styles.text}>Completed</span>
          </>
        ) : (
          <>
            <span className={styles.icon}>📝</span>
            <span className={styles.text}>Mark as Complete</span>
          </>
        )}
      </button>

      {showSuccess && (
        <div className={styles.successMessage}>
          <span>✓</span>
          <span>Progress updated successfully!</span>
        </div>
      )}

      {showError && (
        <div className={styles.errorMessage}>
          <span>✗</span>
          <span>{errorMessage}</span>
        </div>
      )}
    </div>
  );
}

import React from 'react';
import styles from './DifficultyBadge.module.css';

type DifficultyLevel = 'beginner' | 'intermediate' | 'advanced';

interface DifficultyBadgeProps {
  level: DifficultyLevel;
  size?: 'small' | 'medium' | 'large';
  className?: string;
}

const difficultyConfig = {
  beginner: {
    label: 'Beginner',
    icon: '🌱',
    description: 'No prior knowledge required'
  },
  intermediate: {
    label: 'Intermediate',
    icon: '🚀',
    description: 'Some programming experience helpful'
  },
  advanced: {
    label: 'Advanced',
    icon: '⚡',
    description: 'Strong technical background recommended'
  }
};

export default function DifficultyBadge({
  level,
  size = 'medium',
  className
}: DifficultyBadgeProps): JSX.Element {
  const config = difficultyConfig[level];

  return (
    <span
      className={`${styles.badge} ${styles[level]} ${styles[size]} ${className || ''}`}
      title={config.description}
      role="img"
      aria-label={`Difficulty: ${config.label}`}
    >
      <span className={styles.icon}>{config.icon}</span>
      <span className={styles.label}>{config.label}</span>
    </span>
  );
}

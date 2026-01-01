import React from 'react';
import DocItemLayout from '@theme-original/DocItem/Layout';
import DifficultyBadge from '../../../components/common/DifficultyBadge';
import { usePersonalization } from '../../../contexts/PersonalizationContext';
import styles from './styles.module.css';

export default function DocItemLayoutWrapper(props) {
  const { shouldShowContent, isPersonalizationEnabled, experienceLevel } = usePersonalization();

  // Get difficulty from frontmatter via props
  const difficulty = props?.content?.frontMatter?.difficulty as 'beginner' | 'intermediate' | 'advanced' | undefined;

  // Check if content should be visible based on user's experience level
  const isVisible = !difficulty || shouldShowContent(difficulty);

  // If personalization is enabled and content shouldn't be shown, display a message
  if (isPersonalizationEnabled && !isVisible && experienceLevel !== 'all') {
    return (
      <div className={styles.contentHidden}>
        <div className={styles.hiddenMessage}>
          <span className={styles.icon}>🔒</span>
          <h2>Content Filtered</h2>
          <p>
            This {difficulty} level content is filtered based on your current experience level ({experienceLevel}).
          </p>
          <p className={styles.helpText}>
            You can adjust your content preferences using the <strong>Content Level</strong> button in the bottom right corner.
          </p>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.docItemWrapper}>
      {difficulty && (
        <div className={styles.difficultyContainer}>
          <DifficultyBadge level={difficulty} size="medium" />
        </div>
      )}
      <DocItemLayout {...props} />
    </div>
  );
}

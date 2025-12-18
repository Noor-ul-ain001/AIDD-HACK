import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import { useAuth } from './AuthContext';

type ExperienceLevel = 'beginner' | 'intermediate' | 'advanced' | 'all';

type ContentDifficulty = 'beginner' | 'intermediate' | 'advanced';

interface PersonalizationContextType {
  experienceLevel: ExperienceLevel;
  setExperienceLevel: (level: ExperienceLevel) => void;
  shouldShowContent: (contentDifficulty: ContentDifficulty | ContentDifficulty[]) => boolean;
  isPersonalizationEnabled: boolean;
  togglePersonalization: () => void;
  learningGoals: string[];
}

const PersonalizationContext = createContext<PersonalizationContextType | undefined>(undefined);

export function PersonalizationProvider({ children }: { children: ReactNode }) {
  const { user, isAuthenticated } = useAuth();
  const [experienceLevel, setExperienceLevelState] = useState<ExperienceLevel>('all');
  const [isPersonalizationEnabled, setIsPersonalizationEnabled] = useState(true);
  const [learningGoals, setLearningGoals] = useState<string[]>([]);

  // Load user's experience level from profile when authenticated
  useEffect(() => {
    if (isAuthenticated && user?.hardware_profile?.experience_level) {
      const level = user.hardware_profile.experience_level as ExperienceLevel;
      console.log('[PERSONALIZATION] User experience level:', level);
      setExperienceLevelState(level);
    } else if (!isAuthenticated) {
      // Default to 'all' for unauthenticated users
      setExperienceLevelState('all');
    }

    if (isAuthenticated && user?.learning_goals) {
      setLearningGoals(user.learning_goals);
    }
  }, [isAuthenticated, user]);

  const setExperienceLevel = (level: ExperienceLevel) => {
    console.log('[PERSONALIZATION] Experience level changed to:', level);
    setExperienceLevelState(level);
    // Optionally save to localStorage for persistence
    if (typeof window !== 'undefined') {
      localStorage.setItem('experience_level_override', level);
    }
  };

  const togglePersonalization = () => {
    setIsPersonalizationEnabled(prev => {
      const newValue = !prev;
      console.log('[PERSONALIZATION] Personalization', newValue ? 'enabled' : 'disabled');
      if (typeof window !== 'undefined') {
        localStorage.setItem('personalization_enabled', String(newValue));
      }
      return newValue;
    });
  };

  /**
   * Determines if content should be shown based on user's experience level
   * @param contentDifficulty - The difficulty level(s) of the content
   * @returns true if the content should be displayed
   */
  const shouldShowContent = (contentDifficulty: ContentDifficulty | ContentDifficulty[]): boolean => {
    // If personalization is disabled, show all content
    if (!isPersonalizationEnabled) {
      return true;
    }

    // If user hasn't set a level or is viewing 'all', show everything
    if (experienceLevel === 'all') {
      return true;
    }

    // Handle array of difficulties (content suitable for multiple levels)
    if (Array.isArray(contentDifficulty)) {
      return contentDifficulty.includes(experienceLevel as ContentDifficulty);
    }

    // Handle single difficulty
    const levelHierarchy: { [key: string]: number } = {
      beginner: 1,
      intermediate: 2,
      advanced: 3
    };

    const userLevel = levelHierarchy[experienceLevel] || 1;
    const contentLevel = levelHierarchy[contentDifficulty] || 1;

    // Show content at or slightly below user's level
    // Beginners: only beginner content
    // Intermediate: beginner and intermediate content
    // Advanced: all content
    if (experienceLevel === 'beginner') {
      return contentDifficulty === 'beginner';
    } else if (experienceLevel === 'intermediate') {
      return contentDifficulty === 'beginner' || contentDifficulty === 'intermediate';
    } else if (experienceLevel === 'advanced') {
      return true; // Show all content for advanced users
    }

    return contentLevel <= userLevel;
  };

  const value: PersonalizationContextType = {
    experienceLevel,
    setExperienceLevel,
    shouldShowContent,
    isPersonalizationEnabled,
    togglePersonalization,
    learningGoals
  };

  return (
    <PersonalizationContext.Provider value={value}>
      {children}
    </PersonalizationContext.Provider>
  );
}

export function usePersonalization() {
  const context = useContext(PersonalizationContext);
  if (context === undefined) {
    throw new Error('usePersonalization must be used within a PersonalizationProvider');
  }
  return context;
}

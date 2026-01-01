import React, { useEffect } from 'react';
import { useTranslation as useI18next } from 'react-i18next';
import clsx from 'clsx';
import styles from './TranslationButton.module.css';
import { useTranslation } from '../../contexts/TranslationContext';

type TranslationButtonProps = {
  chapterId?: string;
  onLanguageChange?: (language: 'en' | 'ur') => void;
  className?: string;
};

export default function TranslationButton({
  chapterId,
  onLanguageChange,
  className
}: TranslationButtonProps): JSX.Element {
  const { t, i18n } = useI18next();
  const { currentLanguage, changeLanguage, translateContent } = useTranslation();

  // Sync with i18n language on mount
  useEffect(() => {
    const savedLanguage = localStorage.getItem('preferred_language');
    if (savedLanguage && (savedLanguage === 'ur' || savedLanguage === 'en')) {
      i18n.changeLanguage(savedLanguage);
    }
  }, [i18n]);

  const toggleLanguage = async () => {
    const newLanguage = currentLanguage === 'en' ? 'ur' : 'en';

    // Change UI language
    await changeLanguage(newLanguage);

    // Save preference to localStorage
    localStorage.setItem('preferred_language', newLanguage);

    // Notify parent component
    onLanguageChange?.(newLanguage as 'en' | 'ur');

    // Note: Content translation is automatically handled by DocItemContent component
    // based on the current page's chapterId

    console.log(`Language changed to: ${newLanguage}`);
  };

  return (
    <div className={clsx(styles.buttonContainer, className)}>
      <button
        className={clsx(
          styles.button,
          currentLanguage === 'ur' ? styles.urdu : styles.english
        )}
        onClick={toggleLanguage}
        title={currentLanguage === 'en' ? 'Switch to Urdu' : 'Switch to English'}
      >
        <span className={styles.icon}>🌐</span>
        <span>
          {currentLanguage === 'en'
            ? t('button.switchToUrdu')
            : t('button.switchToEnglish')}
        </span>
      </button>
    </div>
  );
}

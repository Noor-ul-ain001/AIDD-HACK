import React, { useState, useRef, useEffect } from 'react';
import clsx from 'clsx';
import { useTranslation as useI18next } from 'react-i18next';
import ChatbotWidget from '../../chat/ChatbotWidget';
import UserAuthButton from '../../auth/UserAuthButton';
import TranslationButton from '../../buttons/TranslationButton';
import PersonalizationModal from '../../personalization/PersonalizationModal';
import { useTranslation } from '../../../contexts/TranslationContext';
import styles from './FloatingChatIcon.module.css';

type FloatingChatIconProps = {
  chapterId?: string;
  className?: string;
};

export default function FloatingChatIcon({
  chapterId = 'default',
  className
}: FloatingChatIconProps): JSX.Element {
  const [isOpen, setIsOpen] = useState(false);
  const [activeTab, setActiveTab] = useState<'chat' | 'auth' | 'translate' | 'personalize'>('chat');
  const [showPersonalizationModal, setShowPersonalizationModal] = useState(false);
  const containerRef = useRef<HTMLDivElement>(null);
  const { isTranslating, translationError } = useTranslation();
  const { t } = useI18next();

  // Close the panel when clicking outside
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (containerRef.current && !containerRef.current.contains(event.target as Node)) {
        if (isOpen) {
          setIsOpen(false);
        }
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [isOpen]);

  const togglePanel = () => {
    setIsOpen(!isOpen);
    if (!isOpen) {
      // Default to chat when opening
      setActiveTab('chat');
    }
  };

  const handleLanguageChange = (language: 'en' | 'ur') => {
    console.log(`Language changed to: ${language}`);
    // The translation context handles the actual language change
  };

  const handlePersonalizeClick = () => {
    setActiveTab('personalize');
    setShowPersonalizationModal(true);
    setIsOpen(true); // Ensure the panel is open when showing personalization modal
  };

  return (
    <div className={clsx(styles.floatingContainer, className)} ref={containerRef}>
      {/* Floating Icon Button */}
      <button
        className={clsx(styles.floatingButton, isOpen && styles.open)}
        onClick={togglePanel}
        aria-label={isOpen ? t('floatingChat.closePanel') : t('floatingChat.openPanel')}
        title={t('floatingChat.aiAssistant')}
      >
        <span className={styles.icon}>🤖</span>
        {!isOpen && (
          <span className={styles.badge}>
            <span className={styles.pulse}></span>
          </span>
        )}
      </button>

      {/* Expanded Panel */}
      {isOpen && (
        <div className={styles.panel}>
          {/* Tab Navigation */}
          <div className={styles.tabNavigation}>
            <button
              className={clsx(styles.tabButton, activeTab === 'chat' && styles.active)}
              onClick={() => setActiveTab('chat')}
              aria-label={t('floatingChat.chatTab')}
            >
              💬
            </button>
            <button
              className={clsx(styles.tabButton, activeTab === 'auth' && styles.active)}
              onClick={() => setActiveTab('auth')}
              aria-label={t('floatingChat.authTab')}
            >
              🔐
            </button>
            <button
              className={clsx(styles.tabButton, activeTab === 'translate' && styles.active)}
              onClick={() => setActiveTab('translate')}
              aria-label={t('floatingChat.translateTab')}
            >
              🌐
            </button>
            <button
              className={clsx(styles.tabButton, activeTab === 'personalize' && styles.active)}
              onClick={handlePersonalizeClick}
              aria-label={t('floatingChat.personalizeTab')}
            >
              🎯
            </button>
          </div>

          {/* Tab Content */}
          <div className={styles.tabContent}>
            {activeTab === 'chat' && (
              <div className={styles.chatContainer}>
                <ChatbotWidget className={styles.chatWidget} />
              </div>
            )}

            {activeTab === 'auth' && (
              <div className={styles.authContainer}>
                <h3>{t('auth.title')}</h3>
                <UserAuthButton />
              </div>
            )}

            {activeTab === 'translate' && (
              <div className={styles.translateContainer}>
                <h3>{t('translation.title')}</h3>
                <p>{t('translation.description')}</p>
                <TranslationButton
                  chapterId={chapterId}
                  onLanguageChange={handleLanguageChange}
                />
                {isTranslating && (
                  <p className={styles.statusMessage}>{t('status.translating')}</p>
                )}
                {translationError && (
                  <p className={styles.errorMessage}>{translationError}</p>
                )}
                <p className={styles.helpText}>
                  {t('translation.helpText')}
                </p>
              </div>
            )}

            {activeTab === 'personalize' && (
              <div className={styles.personalizeContainer}>
                <h3>{t('personalization.title')}</h3>
                <p>{t('personalization.description')}</p>
                <button
                  className={styles.personalizeButton}
                  onClick={handlePersonalizeClick}
                >
                  {t('personalization.buttonText')}
                </button>
                <p className={styles.helpText}>
                  {t('personalization.helpText')}
                </p>
              </div>
            )}
          </div>
        </div>
      )}

      {/* Personalization Modal */}
      {showPersonalizationModal && (
        <PersonalizationModal
          isOpen={showPersonalizationModal}
          onClose={() => setShowPersonalizationModal(false)}
          chapterId={chapterId}
        />
      )}
    </div>
  );
}
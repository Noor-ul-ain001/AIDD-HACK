import React from 'react';
import { AuthProvider } from '../contexts/AuthContext';
import { TranslationProvider } from '../contexts/TranslationContext';
import { PersonalizationProvider } from '../contexts/PersonalizationContext';
import FloatingChatIcon from '../components/common/FloatingChatIcon/FloatingChatIcon';
import PersonalizationPanel from '../components/common/PersonalizationPanel';

// Initialize i18next for custom React components
import '../i18n/config';

// Docusaurus Root wrapper - wraps the entire app
export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <AuthProvider>
      <PersonalizationProvider>
        <TranslationProvider>
          <>
            {children}
            <FloatingChatIcon />
            <PersonalizationPanel />
          </>
        </TranslationProvider>
      </PersonalizationProvider>
    </AuthProvider>
  );
}

import i18n from 'i18next';
import { initReactI18next } from 'react-i18next';
import LanguageDetector from 'i18next-browser-languagedetector';

// Translation resources
import en from './locales/en.json';
import ur from './locales/ur.json';

i18n
  // Detect user language
  .use(LanguageDetector)
  // Pass the i18n instance to react-i18next
  .use(initReactI18next)
  // Initialize i18next
  .init({
    resources: {
      en: {
        translation: en
      },
      ur: {
        translation: ur
      }
    },
    fallbackLng: 'en',
    debug: false,

    // Language detection options
    detection: {
      order: ['localStorage', 'navigator'],
      caches: ['localStorage'],
      lookupLocalStorage: 'preferred_language'
    },

    interpolation: {
      escapeValue: false // React already escapes values
    },

    react: {
      useSuspense: false // Disable suspense for SSR compatibility with Docusaurus
    }
  });

export default i18n;

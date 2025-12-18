import React, { createContext, useContext, useReducer, useCallback } from 'react';
import { useTranslation as useI18next } from 'react-i18next';
import { ContentAPI } from '../lib/api';

// Define types for dynamic content translation state
type TranslationState = {
  isTranslating: boolean;
  translatedContent: string | null;
  preservedCodeBlocks: string[];
  translationError: string | null;
  currentChapterId: string | null;
};

type TranslationAction =
  | { type: 'START_TRANSLATION'; chapterId: string }
  | { type: 'TRANSLATION_SUCCESS'; payload: { translatedContent: string; preservedCodeBlocks: string[] } }
  | { type: 'TRANSLATION_ERROR'; payload: string }
  | { type: 'RESET_TRANSLATION' };

// Initial state
const initialState: TranslationState = {
  isTranslating: false,
  translatedContent: null,
  preservedCodeBlocks: [],
  translationError: null,
  currentChapterId: null
};

// Reducer function
const translationReducer = (state: TranslationState, action: TranslationAction): TranslationState => {
  switch (action.type) {
    case 'START_TRANSLATION':
      return {
        ...state,
        isTranslating: true,
        translationError: null,
        currentChapterId: action.chapterId
      };

    case 'TRANSLATION_SUCCESS':
      return {
        ...state,
        isTranslating: false,
        translatedContent: action.payload.translatedContent,
        preservedCodeBlocks: action.payload.preservedCodeBlocks
      };

    case 'TRANSLATION_ERROR':
      return {
        ...state,
        isTranslating: false,
        translationError: action.payload
      };

    case 'RESET_TRANSLATION':
      return initialState;

    default:
      return state;
  }
};

// Create context
interface TranslationContextType extends TranslationState {
  translateContent: (chapterId: string) => Promise<void>;
  resetTranslation: () => void;
  currentLanguage: string;
  changeLanguage: (lang: string) => Promise<void>;
}

const TranslationContext = createContext<TranslationContextType | undefined>(undefined);

// Provider component
type TranslationProviderProps = {
  children: React.ReactNode;
};

export const TranslationProvider: React.FC<TranslationProviderProps> = ({ children }) => {
  const [state, dispatch] = useReducer(translationReducer, initialState);
  const { i18n } = useI18next();

  /**
   * Translate dynamic content (chapter/page content) via backend API
   * Set ENABLE_MOCK_TRANSLATION=true in .env to test without backend
   */
  const translateContent = useCallback(async (chapterId: string) => {
    dispatch({ type: 'START_TRANSLATION', chapterId });

    try {
      // Check if mock mode is enabled (for testing without backend)
      const useMock = typeof window !== 'undefined' &&
        (window as any).ENABLE_MOCK_TRANSLATION === true;

      if (useMock) {
        // Mock translation for testing
        console.log('🔄 [MOCK] Translating chapter:', chapterId);
        await new Promise(resolve => setTimeout(resolve, 1500)); // Simulate API delay

        const mockTranslatedContent = `
          <div class="urdu-content">
            <h1>یہ ایک ٹیسٹ ترجمہ ہے</h1>
            <p>باب کی شناخت: ${chapterId}</p>
            <p>یہ مواد اردو میں ترجمہ شدہ ہے۔ یہ صرف ایک ٹیسٹ ہے تاکہ آپ دیکھ سکیں کہ ترجمہ کی خصوصیت کام کر رہی ہے یا نہیں۔</p>
            <p>جب حقیقی بیک اینڈ API چل رہی ہو گی، تو یہ مواد اصل ترجمہ شدہ متن سے تبدیل ہو جائے گا۔</p>
            <h2>خصوصیات:</h2>
            <ul>
              <li>دائیں سے بائیں (RTL) سمت</li>
              <li>اردو نستعلیق فونٹ</li>
              <li>مناسب لائن کی اونچائی</li>
            </ul>
            <pre><code>// Code blocks remain in English (LTR)
function example() {
  console.log("This is preserved!");
}</code></pre>
          </div>
        `;

        dispatch({
          type: 'TRANSLATION_SUCCESS',
          payload: {
            translatedContent: mockTranslatedContent,
            preservedCodeBlocks: []
          }
        });
        console.log('✅ [MOCK] Translation completed');
        return;
      }

      // Real API call
      console.log('🔄 Translating chapter:', chapterId, 'to', i18n.language);
      const response = await ContentAPI.translate(chapterId, i18n.language);

      if (response.error) {
        throw new Error(response.error);
      }

      if (response.data) {
        dispatch({
          type: 'TRANSLATION_SUCCESS',
          payload: {
            translatedContent: response.data.translatedContent || '',
            preservedCodeBlocks: response.data.preservedCodeBlocks || []
          }
        });
        console.log('✅ Translation completed successfully');
      }
    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : 'Translation failed. Please try again.';
      console.error('❌ Translation error:', errorMessage);
      dispatch({
        type: 'TRANSLATION_ERROR',
        payload: errorMessage
      });
    }
  }, [i18n.language]);

  /**
   * Reset translation state (e.g., when switching back to English)
   */
  const resetTranslation = useCallback(() => {
    dispatch({ type: 'RESET_TRANSLATION' });
  }, []);

  /**
   * Change UI language using i18next
   */
  const changeLanguage = useCallback(async (lang: string) => {
    await i18n.changeLanguage(lang);

    // Reset dynamic content translation when switching back to English
    if (lang === 'en') {
      resetTranslation();
    }
  }, [i18n, resetTranslation]);

  const value: TranslationContextType = {
    ...state,
    translateContent,
    resetTranslation,
    currentLanguage: i18n.language,
    changeLanguage
  };

  return (
    <TranslationContext.Provider value={value}>
      {children}
    </TranslationContext.Provider>
  );
};

// Custom hook to use the translation context
export const useTranslation = (): TranslationContextType => {
  const context = useContext(TranslationContext);

  if (context === undefined) {
    throw new Error('useTranslation must be used within a TranslationProvider');
  }

  return context;
};

// Export a hook that combines both i18next and custom translation
export const useFullTranslation = () => {
  const customTranslation = useTranslation();
  const i18nextTranslation = useI18next();

  return {
    ...customTranslation,
    t: i18nextTranslation.t,
    i18n: i18nextTranslation.i18n
  };
};

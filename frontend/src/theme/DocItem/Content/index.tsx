/**
 * Swizzled DocItem/Content component to support translation
 * Renders translated content when available, otherwise renders original
 */

import React, { type ReactNode, useEffect } from 'react';
import clsx from 'clsx';
import { ThemeClassNames } from '@docusaurus/theme-common';
import { useDoc } from '@docusaurus/plugin-content-docs/client';
import Heading from '@theme/Heading';
import MDXContent from '@theme/MDXContent';
import ProgressButton from '../../../components/buttons/ProgressButton';
import { useTranslation } from '../../../contexts/TranslationContext';
// import '../../../css/urdu.css';

// Props type for this component
interface Props {
  readonly children: ReactNode;
}

/**
 * Title can be declared inside md content or declared through
 * front matter and added manually. To make both cases consistent,
 * the added title is added under the same div.markdown block
 * See https://github.com/facebook/docusaurus/pull/4882#issuecomment-853021120
 *
 * We render a "synthetic title" if:
 * - user doesn't ask to hide it with front matter
 * - the markdown content does not already contain a top-level h1 heading
 */
function useSyntheticTitle(): string | null {
  const { metadata, frontMatter, contentTitle } = useDoc();
  const shouldRender =
    !frontMatter.hide_title && typeof contentTitle === 'undefined';
  if (!shouldRender) {
    return null;
  }
  return metadata.title;
}

export default function DocItemContent({ children }: Props): ReactNode {
  const syntheticTitle = useSyntheticTitle();
  const { metadata } = useDoc();
  const {
    translatedContent,
    currentLanguage,
    translateContent,
    isTranslating,
    currentChapterId
  } = useTranslation();

  // Get chapter ID from metadata
  const chapterId = metadata.id || metadata.slug || 'default';

  // Auto-translate content when language changes to Urdu or when navigating to a new page
  useEffect(() => {
    const needsTranslation = currentLanguage === 'ur' &&
                            !isTranslating &&
                            (currentChapterId !== chapterId || !translatedContent);

    if (needsTranslation) {
      translateContent(chapterId);
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
    // Note: translateContent is intentionally excluded from dependencies to prevent infinite loop
    // The effect is properly controlled by currentLanguage, chapterId, currentChapterId, translatedContent, and isTranslating
  }, [currentLanguage, chapterId, currentChapterId, translatedContent, isTranslating]);

  // Determine if we should show translated content (only if it's for the current chapter)
  const showTranslated = currentLanguage === 'ur' && translatedContent && currentChapterId === chapterId;

  return (
    <>
      <div
        className={clsx(
          ThemeClassNames.docs.docMarkdown,
          'markdown',
          showTranslated && 'urdu-content'
        )}
        dir={currentLanguage === 'ur' ? 'rtl' : 'ltr'}
      >
        {syntheticTitle && (
          <header>
            <Heading as="h1">{syntheticTitle}</Heading>
          </header>
        )}

        {isTranslating && currentLanguage === 'ur' && (
          <div className="translation-notice">
            <div className="translating-indicator">
              <span className="spinner"></span>
              <span>ترجمہ جاری ہے...</span>
            </div>
          </div>
        )}

        {showTranslated ? (
          <div
            className="translated-content"
            dangerouslySetInnerHTML={{ __html: translatedContent }}
          />
        ) : (
          <MDXContent>{children}</MDXContent>
        )}

        {currentLanguage === 'ur' && !translatedContent && !isTranslating && (
          <div className="translation-notice">
            اس صفحہ کا ترجمہ دستیاب نہیں ہے۔
          </div>
        )}
      </div>

      <div style={{
        marginTop: '3rem',
        paddingTop: '2rem',
        borderTop: '1px solid #e5e7eb',
        display: 'flex',
        justifyContent: 'center'
      }}>
        <ProgressButton chapterId={chapterId} />
      </div>
    </>
  );
}

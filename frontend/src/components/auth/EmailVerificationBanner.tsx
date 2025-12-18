import React, { useState } from 'react';
import { useTranslation } from 'react-i18next';
import { AuthAPI } from '../../lib/api';
import styles from './EmailVerificationBanner.module.css';

interface EmailVerificationBannerProps {
  email: string;
  onVerificationSent?: () => void;
}

export default function EmailVerificationBanner({
  email,
  onVerificationSent
}: EmailVerificationBannerProps): JSX.Element {
  const { t } = useTranslation();
  const [isResending, setIsResending] = useState(false);
  const [resendStatus, setResendStatus] = useState<'idle' | 'success' | 'error'>('idle');
  const [errorMessage, setErrorMessage] = useState('');

  const handleResendEmail = async () => {
    setIsResending(true);
    setResendStatus('idle');
    setErrorMessage('');

    try {
      const response = await AuthAPI.resendVerificationEmail();

      if (response.error) {
        setResendStatus('error');
        setErrorMessage(response.error);
      } else {
        setResendStatus('success');
        onVerificationSent?.();

        // Reset success message after 5 seconds
        setTimeout(() => {
          setResendStatus('idle');
        }, 5000);
      }
    } catch (error) {
      setResendStatus('error');
      setErrorMessage('Failed to resend verification email. Please try again.');
    } finally {
      setIsResending(false);
    }
  };

  return (
    <div className={styles.banner}>
      <div className={styles.content}>
        <div className={styles.iconSection}>
          <span className={styles.icon}>📧</span>
        </div>
        <div className={styles.textSection}>
          <h4 className={styles.title}>{t('auth.emailVerification.title')}</h4>
          <p className={styles.message}>
            {t('auth.emailVerification.message', { email })}
          </p>
          {resendStatus === 'success' && (
            <p className={styles.successMessage}>
              ✓ {t('auth.emailVerification.resendSuccess')}
            </p>
          )}
          {resendStatus === 'error' && (
            <p className={styles.errorMessage}>
              {errorMessage || t('auth.emailVerification.resendError')}
            </p>
          )}
        </div>
        <div className={styles.actionSection}>
          <button
            onClick={handleResendEmail}
            disabled={isResending || resendStatus === 'success'}
            className={styles.resendButton}
          >
            {isResending ? (
              <>
                <span className={styles.spinner}></span>
                {t('auth.emailVerification.resending')}
              </>
            ) : (
              t('auth.emailVerification.resendButton')
            )}
          </button>
        </div>
      </div>
    </div>
  );
}

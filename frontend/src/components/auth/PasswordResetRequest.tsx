import React, { useState } from 'react';
import { useTranslation } from 'react-i18next';
import clsx from 'clsx';
import { AuthAPI } from '../../lib/api';
import styles from './PasswordResetRequest.module.css';

interface PasswordResetRequestProps {
  onCancel?: () => void;
  onSuccess?: () => void;
}

export default function PasswordResetRequest({
  onCancel,
  onSuccess
}: PasswordResetRequestProps): JSX.Element {
  const { t } = useTranslation();
  const [email, setEmail] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState('');
  const [success, setSuccess] = useState(false);

  const validateEmail = (email: string): boolean => {
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    return emailRegex.test(email);
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    // Validation
    if (!email) {
      setError(t('auth.passwordReset.emailRequired'));
      return;
    }

    if (!validateEmail(email)) {
      setError(t('auth.passwordReset.emailInvalid'));
      return;
    }

    setIsLoading(true);
    setError('');

    try {
      const response = await AuthAPI.requestPasswordReset(email);

      if (response.error) {
        setError(response.error);
      } else {
        setSuccess(true);
        onSuccess?.();
      }
    } catch (err) {
      setError(t('auth.passwordReset.requestError'));
    } finally {
      setIsLoading(false);
    }
  };

  if (success) {
    return (
      <div className={styles.container}>
        <div className={styles.successCard}>
          <div className={styles.successIcon}>✓</div>
          <h2>{t('auth.passwordReset.successTitle')}</h2>
          <p className={styles.successMessage}>
            {t('auth.passwordReset.successMessage', { email })}
          </p>
          <p className={styles.helpText}>
            {t('auth.passwordReset.checkSpam')}
          </p>
          <button
            onClick={onCancel}
            className={styles.backButton}
          >
            {t('auth.passwordReset.backToLogin')}
          </button>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.container}>
      <div className={styles.card}>
        <h2>{t('auth.passwordReset.title')}</h2>
        <p className={styles.description}>
          {t('auth.passwordReset.description')}
        </p>

        <form onSubmit={handleSubmit} className={styles.form}>
          <div className={styles.inputGroup}>
            <label htmlFor="email" className={styles.label}>
              {t('auth.email')}
            </label>
            <input
              type="email"
              id="email"
              value={email}
              onChange={(e) => {
                setEmail(e.target.value);
                setError('');
              }}
              className={clsx(styles.input, error && styles.inputError)}
              placeholder={t('auth.passwordReset.emailPlaceholder')}
              disabled={isLoading}
              autoFocus
            />
            {error && <span className={styles.error}>{error}</span>}
          </div>

          <div className={styles.actions}>
            <button
              type="submit"
              className={styles.submitButton}
              disabled={isLoading}
            >
              {isLoading ? (
                <>
                  <span className={styles.spinner}></span>
                  {t('auth.passwordReset.sending')}
                </>
              ) : (
                t('auth.passwordReset.sendReset')
              )}
            </button>

            {onCancel && (
              <button
                type="button"
                onClick={onCancel}
                className={styles.cancelButton}
                disabled={isLoading}
              >
                {t('common.cancel')}
              </button>
            )}
          </div>
        </form>
      </div>
    </div>
  );
}

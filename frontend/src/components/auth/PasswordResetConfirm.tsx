import React, { useState, useEffect } from 'react';
import { useTranslation } from 'react-i18next';
import clsx from 'clsx';
import { AuthAPI } from '../../lib/api';
import styles from './PasswordResetConfirm.module.css';

interface PasswordResetConfirmProps {
  token: string;
  onSuccess?: () => void;
  onError?: (error: string) => void;
}

type PasswordStrength = {
  score: number;
  text: string;
  color: string;
};

export default function PasswordResetConfirm({
  token,
  onSuccess,
  onError
}: PasswordResetConfirmProps): JSX.Element {
  const { t } = useTranslation();
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isVerifying, setIsVerifying] = useState(true);
  const [tokenValid, setTokenValid] = useState(false);
  const [userEmail, setUserEmail] = useState('');
  const [errors, setErrors] = useState<{ password?: string; confirmPassword?: string }>({});
  const [success, setSuccess] = useState(false);
  const [passwordStrength, setPasswordStrength] = useState<PasswordStrength>({
    score: 0,
    text: '',
    color: '#ff6b6b'
  });

  // Verify token on mount
  useEffect(() => {
    const verifyToken = async () => {
      try {
        const response = await AuthAPI.verifyResetToken(token);

        if (response.data?.valid) {
          setTokenValid(true);
          setUserEmail(response.data.email || '');
        } else {
          setTokenValid(false);
          onError?.(t('auth.passwordReset.invalidToken'));
        }
      } catch (err) {
        setTokenValid(false);
        onError?.(t('auth.passwordReset.verifyError'));
      } finally {
        setIsVerifying(false);
      }
    };

    verifyToken();
  }, [token, t, onError]);

  const checkPasswordStrength = (password: string): PasswordStrength => {
    let score = 0;
    let text = '';
    let color = '#ff6b6b';

    if (!password) {
      return { score: 0, text: '', color };
    }

    // Length check
    if (password.length >= 8) score += 1;
    if (password.length >= 12) score += 1;

    // Complexity checks
    if (/[A-Z]/.test(password)) score += 1;
    if (/[a-z]/.test(password)) score += 1;
    if (/[0-9]/.test(password)) score += 1;
    if (/[^A-Za-z0-9]/.test(password)) score += 1;

    // Determine strength level
    if (score <= 2) {
      text = t('auth.passwordStrength.weak');
      color = '#ff6b6b';
    } else if (score <= 4) {
      text = t('auth.passwordStrength.medium');
      color = '#ffd93d';
    } else {
      text = t('auth.passwordStrength.strong');
      color = '#4FA675';
    }

    return { score, text, color };
  };

  useEffect(() => {
    setPasswordStrength(checkPasswordStrength(password));
  }, [password, t]);

  const validate = (): boolean => {
    const newErrors: { password?: string; confirmPassword?: string } = {};

    if (!password) {
      newErrors.password = t('auth.passwordReset.passwordRequired');
    } else if (password.length < 8) {
      newErrors.password = t('auth.passwordReset.passwordTooShort');
    } else if (passwordStrength.score <= 2) {
      newErrors.password = t('auth.passwordReset.passwordTooWeak');
    }

    if (password !== confirmPassword) {
      newErrors.confirmPassword = t('auth.passwordReset.passwordMismatch');
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!validate()) {
      return;
    }

    setIsLoading(true);

    try {
      const response = await AuthAPI.resetPassword(token, password);

      if (response.error) {
        onError?.(response.error);
      } else {
        setSuccess(true);
        onSuccess?.();

        // Redirect to login after 3 seconds
        setTimeout(() => {
          window.location.href = '/signin';
        }, 3000);
      }
    } catch (err) {
      onError?.(t('auth.passwordReset.resetError'));
    } finally {
      setIsLoading(false);
    }
  };

  if (isVerifying) {
    return (
      <div className={styles.container}>
        <div className={styles.loadingCard}>
          <div className={styles.spinner}></div>
          <p>{t('auth.passwordReset.verifyingToken')}</p>
        </div>
      </div>
    );
  }

  if (!tokenValid) {
    return (
      <div className={styles.container}>
        <div className={styles.errorCard}>
          <div className={styles.errorIcon}>✕</div>
          <h2>{t('auth.passwordReset.invalidTokenTitle')}</h2>
          <p>{t('auth.passwordReset.invalidTokenMessage')}</p>
          <a href="/signin" className={styles.backLink}>
            {t('auth.passwordReset.backToLogin')}
          </a>
        </div>
      </div>
    );
  }

  if (success) {
    return (
      <div className={styles.container}>
        <div className={styles.successCard}>
          <div className={styles.successIcon}>✓</div>
          <h2>{t('auth.passwordReset.resetSuccessTitle')}</h2>
          <p>{t('auth.passwordReset.resetSuccessMessage')}</p>
          <p className={styles.redirectMessage}>
            {t('auth.passwordReset.redirecting')}
          </p>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.container}>
      <div className={styles.card}>
        <h2>{t('auth.passwordReset.setNewPassword')}</h2>
        {userEmail && (
          <p className={styles.emailInfo}>
            {t('auth.passwordReset.resettingFor')} <strong>{userEmail}</strong>
          </p>
        )}

        <form onSubmit={handleSubmit} className={styles.form}>
          <div className={styles.inputGroup}>
            <label htmlFor="password" className={styles.label}>
              {t('auth.passwordReset.newPassword')}
            </label>
            <input
              type="password"
              id="password"
              value={password}
              onChange={(e) => {
                setPassword(e.target.value);
                setErrors({ ...errors, password: undefined });
              }}
              className={clsx(styles.input, errors.password && styles.inputError)}
              placeholder={t('auth.passwordReset.passwordPlaceholder')}
              disabled={isLoading}
            />
            {password && (
              <div className={styles.passwordStrength}>
                <div className={styles.strengthBar}>
                  <div
                    className={styles.strengthFill}
                    style={{
                      width: `${(passwordStrength.score / 6) * 100}%`,
                      background: passwordStrength.color
                    }}
                  />
                </div>
                <div className={styles.strengthText}>
                  {t('auth.passwordStrength.label')}: <strong style={{ color: passwordStrength.color }}>
                    {passwordStrength.text}
                  </strong>
                </div>
              </div>
            )}
            {errors.password && <span className={styles.error}>{errors.password}</span>}
          </div>

          <div className={styles.inputGroup}>
            <label htmlFor="confirmPassword" className={styles.label}>
              {t('auth.passwordReset.confirmPassword')}
            </label>
            <input
              type="password"
              id="confirmPassword"
              value={confirmPassword}
              onChange={(e) => {
                setConfirmPassword(e.target.value);
                setErrors({ ...errors, confirmPassword: undefined });
              }}
              className={clsx(styles.input, errors.confirmPassword && styles.inputError)}
              placeholder={t('auth.passwordReset.confirmPasswordPlaceholder')}
              disabled={isLoading}
            />
            {errors.confirmPassword && <span className={styles.error}>{errors.confirmPassword}</span>}
          </div>

          <button
            type="submit"
            className={styles.submitButton}
            disabled={isLoading}
          >
            {isLoading ? (
              <>
                <span className={styles.spinner}></span>
                {t('auth.passwordReset.resetting')}
              </>
            ) : (
              t('auth.passwordReset.resetPassword')
            )}
          </button>
        </form>
      </div>
    </div>
  );
}

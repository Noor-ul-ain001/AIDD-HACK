// Enhanced SignupForm component with password strength
import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import styles from './SignupForm.module.css';
import { useAuth } from '../../contexts/AuthContext';

type FormData = {
  email: string;
  password: string;
  confirmPassword: string;
};

type PasswordStrength = {
  score: number;
  text: string;
  color: string;
};

export default function SignupForm({ onSignup, className }) {
  const { signup } = useAuth();
  const [formData, setFormData] = useState<FormData>({
    email: '',
    password: '',
    confirmPassword: ''
  });
  const [errors, setErrors] = useState<Partial<FormData>>({});
  const [isLoading, setIsLoading] = useState(false);
  const [passwordStrength, setPasswordStrength] = useState<PasswordStrength>({
    score: 0,
    text: '',
    color: '#ff6b6b'
  });
  const [showSuccess, setShowSuccess] = useState(false);

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
      text = 'Weak';
      color = '#ff6b6b';
    } else if (score <= 4) {
      text = 'Medium';
      color = '#ffd93d';
    } else {
      text = 'Strong';
      color = '#4FA675';
    }
    
    return { score, text, color };
  };

  useEffect(() => {
    setPasswordStrength(checkPasswordStrength(formData.password));
  }, [formData.password]);

  const validate = (): boolean => {
    const newErrors: Partial<FormData> = {};

    if (!formData.email) {
      newErrors.email = 'Email is required';
    } else if (!/^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(formData.email)) {
      newErrors.email = 'Please enter a valid email address';
    } else {
      // Enhanced email validation
      const email = formData.email.toLowerCase();
      const [localPart, domain] = email.split('@');

      // Check for disposable email domains
      const disposableDomains = [
        'tempmail.com', 'throwaway.email', 'guerrillamail.com',
        'mailinator.com', '10minutemail.com', 'trashmail.com',
        'maildrop.cc', 'temp-mail.org', 'yopmail.com', 'fakeinbox.com',
        'getnada.com', 'throwawaymail.com', 'temp-mail.io'
      ];

      if (disposableDomains.includes(domain)) {
        newErrors.email = 'Temporary or disposable email addresses are not allowed';
      }

      // Check for test/dummy domains
      const testDomains = ['localhost', 'localdomain', 'test', 'example', 'invalid', 'localhost.localdomain'];
      if (testDomains.includes(domain) || domain.endsWith('.local') || domain.endsWith('.test') || domain.endsWith('.invalid')) {
        newErrors.email = 'Please use a valid email address (not test domains)';
      }

      // Check for dummy-like local parts
      const dummyPatterns = [
        /^test/, /^dummy/, /^fake/, /^example/, /^sample/,
        /^temp/, /^user\d+/, /^admin/, /^root/, /^email/,
        /^mail/, /^myemail/, /^a+$/, /^x+$/, /^[0-9]+$/
      ];

      const blockedLocalParts = [
        'test', 'admin', 'noreply', 'no-reply', 'dummy', 'fake',
        'sample', 'example', 'user', 'username', 'your-email',
        'your_email', 'email', 'mail', 'contact', 'info'
      ];

      if (blockedLocalParts.includes(localPart)) {
        newErrors.email = 'Please use a valid personal email address';
      }

      for (const pattern of dummyPatterns) {
        if (pattern.test(localPart)) {
          newErrors.email = 'Please use a valid personal email address';
          break;
        }
      }
    }
    
    if (!formData.password) {
      newErrors.password = 'Password is required';
    } else if (formData.password.length < 8) {
      newErrors.password = 'Password must be at least 8 characters';
    } else if (!/[A-Z]/.test(formData.password)) {
      newErrors.password = 'Password must contain at least one uppercase letter';
    } else if (!/[a-z]/.test(formData.password)) {
      newErrors.password = 'Password must contain at least one lowercase letter';
    } else if (!/[0-9]/.test(formData.password)) {
      newErrors.password = 'Password must contain at least one number';
    } else if (!/[^A-Za-z0-9]/.test(formData.password)) {
      newErrors.password = 'Password must contain at least one special character';
    } else if (passwordStrength.score <= 2) {
      newErrors.password = 'Please use a stronger password';
    }
    
    if (formData.password !== formData.confirmPassword) {
      newErrors.confirmPassword = 'Passwords do not match';
    }
    
    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
    
    // Clear error when user starts typing
    if (errors[name as keyof FormData]) {
      setErrors(prev => ({
        ...prev,
        [name]: undefined
      }));
    }
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!validate()) {
      return;
    }

    console.log('[SIGNUP FORM] Starting signup process for:', formData.email);
    setIsLoading(true);

    try {
      console.log('[SIGNUP FORM] Calling backend signup API...');
      const result = await signup(formData.email, formData.password, false);
      console.log('[SIGNUP FORM] Backend response:', result);

      if (!result.success) {
        // Parse different types of errors returned from backend
        console.log('[SIGNUP FORM] Signup failed with error:', result.error);
        let errorField = 'email'; // Default to email field
        let errorMessage = result.error || 'Signup failed. Please try again.';

        // Handle specific error cases
        if (result.error?.toLowerCase().includes('already exists') ||
            result.error?.toLowerCase().includes('already registered')) {
          errorField = 'email';
          errorMessage = 'This email is already registered. Please sign in or use a different email.';
        } else if (result.error?.toLowerCase().includes('password')) {
          errorField = 'password';
        } else if (result.error?.toLowerCase().includes('email')) {
          errorField = 'email';
        }

        setErrors({ [errorField]: errorMessage });
        setIsLoading(false);
        console.log('[SIGNUP FORM] Staying on signup form due to error');
        return; // Stop here - do not proceed to profile
      }

      // Signup successful - proceed to profile questionnaire
      console.log('[SIGNUP FORM] Signup successful! Calling parent onSignup callback...');
      onSignup?.(formData);
      console.log('[SIGNUP FORM] Parent callback completed');
    } catch (error) {
      console.error('Signup error:', error);
      setErrors({ email: 'An unexpected error occurred. Please try again.' });
    } finally {
      setIsLoading(false);
    }
  };

  if (showSuccess) {
    return (
      <div className={clsx(styles.signupForm, styles.success)}>
        <div className={styles.successIcon}>✓</div>
        <h3>Account Created Successfully!</h3>
        <p>Redirecting to your dashboard...</p>
      </div>
    );
  }

  return (
    <div className={clsx(styles.signupForm, className)}>
      <h2>Create Account</h2>
      <form onSubmit={handleSubmit} className={styles.form}>
        <div className={styles.inputGroup}>
          <label htmlFor="email" className={styles.label}>Email</label>
          <input
            type="email"
            id="email"
            name="email"
            value={formData.email}
            onChange={handleChange}
            className={clsx(styles.input, errors.email && styles.inputError)}
            placeholder="your.email@example.com"
          />
          {errors.email && <span className={styles.error}>{errors.email}</span>}
        </div>
        
        <div className={styles.inputGroup}>
          <label htmlFor="password" className={styles.label}>Password</label>
          <input
            type="password"
            id="password"
            name="password"
            value={formData.password}
            onChange={handleChange}
            className={clsx(styles.input, errors.password && styles.inputError)}
            placeholder="At least 8 characters"
          />
          {formData.password && (
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
                Password strength: <strong style={{ color: passwordStrength.color }}>
                  {passwordStrength.text}
                </strong>
              </div>
            </div>
          )}

          {/* Password Requirements */}
          {formData.password && (
            <div className={styles.passwordRequirements}>
              <div className={styles.requirementItem}>
                <span className={formData.password.length >= 8 ? styles.met : styles.unmet}>✓</span>
                <span>At least 8 characters</span>
              </div>
              <div className={styles.requirementItem}>
                <span className={/[A-Z]/.test(formData.password) ? styles.met : styles.unmet}>✓</span>
                <span>One uppercase letter</span>
              </div>
              <div className={styles.requirementItem}>
                <span className={/[a-z]/.test(formData.password) ? styles.met : styles.unmet}>✓</span>
                <span>One lowercase letter</span>
              </div>
              <div className={styles.requirementItem}>
                <span className={/[0-9]/.test(formData.password) ? styles.met : styles.unmet}>✓</span>
                <span>One number</span>
              </div>
              <div className={styles.requirementItem}>
                <span className={/[^A-Za-z0-9]/.test(formData.password) ? styles.met : styles.unmet}>✓</span>
                <span>One special character</span>
              </div>
            </div>
          )}
          {errors.password && <span className={styles.error}>{errors.password}</span>}
        </div>
        
        <div className={styles.inputGroup}>
          <label htmlFor="confirmPassword" className={styles.label}>Confirm Password</label>
          <input
            type="password"
            id="confirmPassword"
            name="confirmPassword"
            value={formData.confirmPassword}
            onChange={handleChange}
            className={clsx(styles.input, errors.confirmPassword && styles.inputError)}
            placeholder="Confirm your password"
          />
          {errors.confirmPassword && <span className={styles.error}>{errors.confirmPassword}</span>}
        </div>
        
        <div className={styles.terms}>
          By signing up, you agree to our{' '}
          <a href="/terms">Terms of Service</a> and{' '}
          <a href="/privacy">Privacy Policy</a>.
        </div>
        
        <button 
          type="submit" 
          className={styles.submitButton}
          disabled={isLoading}
        >
          {isLoading ? (
            <>
              <span className={styles.spinner}></span>
              Creating Account...
            </>
          ) : (
            'Sign Up'
          )}
        </button>
      </form>
    </div>
  );
}
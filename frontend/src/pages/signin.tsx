import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import clsx from 'clsx';
import styles from './signin.module.css';
import { useAuth } from '../contexts/AuthContext';

type FormData = {
  email: string;
  password: string;
  rememberMe: boolean;
};

export default function SigninPage(): JSX.Element {
  const { login, isAuthenticated, isLoading: authLoading } = useAuth();
  const [formData, setFormData] = useState<FormData>({
    email: '',
    password: '',
    rememberMe: false
  });
  const [errors, setErrors] = useState<Partial<FormData>>({});
  const [isLoading, setIsLoading] = useState(false);

  // Redirect to dashboard if already authenticated
  useEffect(() => {
    console.log('[SIGNIN PAGE] Auth status:', { isAuthenticated, authLoading });
    if (!authLoading && isAuthenticated) {
      console.log('[SIGNIN PAGE] User already authenticated, redirecting to dashboard');
      if (typeof window !== 'undefined') {
        window.location.href = '/dashboard';
      }
    }
  }, [isAuthenticated, authLoading]);

  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value, type, checked } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: type === 'checkbox' ? checked : value
    }));

    // Clear error when user starts typing
    if (errors[name as keyof FormData]) {
      setErrors(prev => ({
        ...prev,
        [name]: undefined
      }));
    }
  };

  const validate = (): boolean => {
    const newErrors: Partial<FormData> = {};

    if (!formData.email) {
      newErrors.email = 'Email is required';
    } else if (!/^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(formData.email)) {
      newErrors.email = 'Please enter a valid email address';
    }

    if (!formData.password) {
      newErrors.password = 'Password is required';
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
      const result = await login(formData.email, formData.password);

      if (!result.success) {
        // Parse different types of errors returned from backend
        let errorField = 'email'; // Default to email field
        let errorMessage = result.error || 'Login failed. Please try again.';

        // Handle specific error cases
        if (result.error?.toLowerCase().includes('invalid credentials') ||
            result.error?.toLowerCase().includes('invalid email or password')) {
          // Show error on email field but keep password field visible
          errorField = 'email';
          errorMessage = 'Invalid email or password. Please check your credentials and try again.';
        } else if (result.error?.toLowerCase().includes('password')) {
          errorField = 'password';
        } else if (result.error?.toLowerCase().includes('email')) {
          errorField = 'email';
        }

        setErrors({ [errorField]: errorMessage });
      } else {
        // Check if the API returned information about email verification
        if (result.data?.email_verified === false) {
          // Email not verified, show warning but still allow login
          console.log('[SIGNIN PAGE] Login successful but email not verified');
          alert(result.data.message || 'Please verify your email address for full access');
        }

        // Redirect after successful login
        console.log('[SIGNIN PAGE] Login successful, redirecting to dashboard');
        // Add small delay to ensure state is updated
        setTimeout(() => {
          window.location.href = '/dashboard';
        }, 500);
      }
    } catch (error) {
      console.error('[SIGNIN PAGE] Login error:', error);
      setErrors({ email: 'An unexpected error occurred. Please try again.' });
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <Layout title="Sign In" description="Sign in to your Physical AI & Humanoid Robotics account">
      <main className={styles.main}>
        <div className={styles.container}>
          <div className={styles.formContainer}>
            <h1>Sign In</h1>
            <p className={styles.subtitle}>
              Access your Physical AI & Humanoid Robotics account
            </p>
            
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
                  placeholder="Your password"
                />
                {errors.password && <span className={styles.error}>{errors.password}</span>}
              </div>
              
              <div className={styles.checkboxGroup}>
                <label className={styles.checkboxLabel}>
                  <input
                    type="checkbox"
                    name="rememberMe"
                    checked={formData.rememberMe}
                    onChange={handleChange}
                    className={styles.checkbox}
                  />
                  <span>Remember me</span>
                </label>
                
                <a href="/reset-password" className={styles.forgotPassword}>
                  Forgot password?
                </a>
              </div>
              
              <button 
                type="submit" 
                className={styles.submitButton}
              >
                Sign In
              </button>
            </form>
            
            <div className={styles.signupLink}>
              Don't have an account? <a href="/signup">Sign up here</a>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}
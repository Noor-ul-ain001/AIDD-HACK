import React, { useState } from 'react';
import Layout from '@theme/Layout';
import SignupForm from '../components/auth/SignupForm';
import EmailVerificationBanner from '../components/auth/EmailVerificationBanner';
import styles from './signup.module.css';

type FormData = {
  email: string;
  password: string;
  confirmPassword: string;
};

export default function SignupPage(): JSX.Element {
  const [step, setStep] = useState<'signup' | 'verification'>('signup');
  const [userEmail, setUserEmail] = useState<string>('');

  const handleSignup = async (formData: FormData) => {
    // SignupForm has already validated and created the account successfully
    // This callback is only called when signup succeeds
    console.log('[SIGNUP FLOW] Signup successful, showing email verification');
    console.log('[SIGNUP FLOW] User email:', formData.email);
    setUserEmail(formData.email);
    setStep('verification');
  };

  const handleProfileSubmit = async (profileData: ProfileData) => {
    // Save profile data and then log the user in
    console.log('[SIGNUP FLOW] Profile submitted:', profileData);

    // Show completion state first
    console.log('[SIGNUP FLOW] Setting step to complete');
    setStep('complete');

    // Now log the user in with their credentials
    if (userFormData) {
      try {
        console.log('[SIGNUP FLOW] Logging in user:', userFormData.email);
        const loginResult = await login(userFormData.email, userFormData.password);

        if (loginResult.success) {
          console.log('[SIGNUP FLOW] Login successful, saving profile data...');

          // Save profile data to backend
          const { AuthAPI } = await import('../lib/api');
          const profileUpdate = await AuthAPI.updateProfile({
            hardware_profile: {
              gpu: profileData.hardwareProfile.gpu,
              robotics_kit: profileData.hardwareProfile.roboticsKit,
              experience_level: profileData.hardwareProfile.experienceLevel
            },
            learning_goals: profileData.learningGoals
          });

          if (profileUpdate.error) {
            console.error('[SIGNUP FLOW] Failed to save profile:', profileUpdate.error);
          } else {
            console.log('[SIGNUP FLOW] Profile saved successfully');
          }

          console.log('[SIGNUP FLOW] Will redirect in 3 seconds');
          // Redirect to dashboard after successful login
          setTimeout(() => {
            console.log('[SIGNUP FLOW] Redirecting to dashboard');
            window.location.href = '/';
          }, 3000);
        } else {
          console.error('[SIGNUP FLOW] Login failed:', loginResult.error);
        }
      } catch (error) {
        console.error('[SIGNUP FLOW] Login after profile submission failed:', error);
      }
    } else {
      console.error('[SIGNUP FLOW] ERROR: No user form data available!');
    }
  };

  return (
    <Layout title="Sign Up" description="Create your Physical AI & Humanoid Robotics account">
      <main className={styles.main}>
        <div className={styles.container}>
          {step === 'signup' && (
            <div className={styles.stepContainer}>
              <h1>Create Your Account</h1>
              <p className={styles.subtitle}>
                Join the Physical AI & Humanoid Robotics educational platform
              </p>
              <SignupForm onSignup={handleSignup} />
              <div className={styles.loginLink}>
                Already have an account? <a href="/signin">Sign in here</a>
              </div>
            </div>
          )}

          {step === 'loading' && (
            <div className={styles.stepContainer}>
              <h1>Creating Your Account</h1>
              <p className={styles.subtitle}>
                Please wait while we set up your profile...
              </p>
              <div className={styles.loadingState}>
                <div className={styles.spinner}></div>
                <p>Securing your account</p>
              </div>
            </div>
          )}

          {step === 'profile' && (
            <div className={styles.stepContainer}>
              <h1>Welcome! Complete Your Profile</h1>
              <p className={styles.subtitle}>
                Help us personalize your learning experience
              </p>
              <ProfileQuestionnaire
                onSubmit={handleProfileSubmit}
                initialData={{
                  hardwareProfile: {
                    gpu: '',
                    roboticsKit: '',
                    experienceLevel: ''
                  },
                  learningGoals: []
                }}
              />
            </div>
          )}

          {step === 'complete' && (
            <div className={styles.stepContainer}>
              <h1>Account Created Successfully!</h1>
              <p className={styles.subtitle}>
                Welcome to the Physical AI & Humanoid Robotics platform.
              </p>
              <div className={styles.successMessage}>
                <p>Your account has been created and your profile is set up.</p>
                <p>You can now access the curriculum and start learning.</p>
              </div>
              <a href="/dashboard" className={styles.continueButton}>
                Continue to Dashboard
              </a>
            </div>
          )}
        </div>
      </main>
    </Layout>
  );
}
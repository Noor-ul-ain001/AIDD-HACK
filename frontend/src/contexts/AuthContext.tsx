import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import { AuthAPI, UserProfile, TokenManager } from '../lib/api';

interface AuthContextType {
  user: UserProfile | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  login: (email: string, password: string) => Promise<{ success: boolean; error?: string }>;
  signup: (email: string, password: string, autoLogin?: boolean) => Promise<{ success: boolean; error?: string }>;
  logout: () => void;
  refreshUser: () => Promise<void>;
  requestEmailVerification: (email: string) => Promise<{ success: boolean; error?: string }>;
  resendVerificationEmail: () => Promise<{ success: boolean; error?: string }>;
  requestPasswordReset: (email: string) => Promise<{ success: boolean; error?: string }>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export function AuthProvider({ children }: { children: ReactNode }) {
  const [user, setUser] = useState<UserProfile | null>(null);
  const [isLoading, setIsLoading] = useState(true);

  // Check if user is already logged in on mount
  useEffect(() => {
    const initAuth = async () => {
      console.log('[AUTH CONTEXT] Initializing authentication...');
      const token = TokenManager.getToken();
      console.log('[AUTH CONTEXT] Token found:', !!token);
      if (token) {
        await refreshUser();
      } else {
        console.log('[AUTH CONTEXT] No token found, user not authenticated');
        setIsLoading(false);
      }
    };

    initAuth();
  }, []);

  const refreshUser = async () => {
    console.log('[AUTH CONTEXT] Refreshing user data...');
    setIsLoading(true);
    try {
      const response = await AuthAPI.getCurrentUser();
      console.log('[AUTH CONTEXT] getCurrentUser response:', { hasData: !!response.data, error: response.error });
      if (response.data) {
        console.log('[AUTH CONTEXT] User data fetched successfully:', response.data.email);
        setUser(response.data);
      } else {
        // Token is invalid, clear it
        console.log('[AUTH CONTEXT] No user data, clearing token');
        TokenManager.removeToken();
        setUser(null);
      }
    } catch (error) {
      console.error('[AUTH CONTEXT] Error refreshing user:', error);
      TokenManager.removeToken();
      setUser(null);
    } finally {
      console.log('[AUTH CONTEXT] Refresh complete, isAuthenticated:', !!user);
      setIsLoading(false);
    }
  };

  const login = async (
    email: string,
    password: string
  ): Promise<{ success: boolean; error?: string }> => {
    console.log('[AUTH CONTEXT] Login attempt for:', email);
    try {
      const response = await AuthAPI.login({ email, password });
      console.log('[AUTH CONTEXT] Login API response:', { success: !response.error, error: response.error });

      if (response.error) {
        console.log('[AUTH CONTEXT] Login failed:', response.error);
        return { success: false, error: response.error };
      }

      // Fetch user profile after successful login
      console.log('[AUTH CONTEXT] Login successful, fetching user profile...');
      await refreshUser();
      console.log('[AUTH CONTEXT] Login complete, user authenticated');
      return { success: true };
    } catch (error) {
      console.error('[AUTH CONTEXT] Login error:', error);
      return {
        success: false,
        error: error instanceof Error ? error.message : 'Login failed'
      };
    }
  };

  const signup = async (
    email: string,
    password: string,
    autoLogin: boolean = false
  ): Promise<{ success: boolean; error?: string }> => {
    try {
      const response = await AuthAPI.signup({ email, password });

      if (response.error) {
        return { success: false, error: response.error };
      }

      // Only auto-login if explicitly requested
      if (autoLogin) {
        return await login(email, password);
      }

      return { success: true };
    } catch (error) {
      return {
        success: false,
        error: error instanceof Error ? error.message : 'Signup failed'
      };
    }
  };

  const logout = () => {
    AuthAPI.logout();
    setUser(null);
  };

  const requestEmailVerification = async (
    email: string
  ): Promise<{ success: boolean; error?: string }> => {
    try {
      const response = await AuthAPI.requestEmailVerification(email);

      if (response.error) {
        return { success: false, error: response.error };
      }

      return { success: true };
    } catch (error) {
      return {
        success: false,
        error: error instanceof Error ? error.message : 'Failed to send verification email'
      };
    }
  };

  const resendVerificationEmail = async (): Promise<{ success: boolean; error?: string }> => {
    try {
      const response = await AuthAPI.resendVerificationEmail();

      if (response.error) {
        return { success: false, error: response.error };
      }

      return { success: true };
    } catch (error) {
      return {
        success: false,
        error: error instanceof Error ? error.message : 'Failed to resend verification email'
      };
    }
  };

  const requestPasswordReset = async (
    email: string
  ): Promise<{ success: boolean; error?: string }> => {
    try {
      const response = await AuthAPI.requestPasswordReset(email);

      if (response.error) {
        return { success: false, error: response.error };
      }

      return { success: true };
    } catch (error) {
      return {
        success: false,
        error: error instanceof Error ? error.message : 'Failed to request password reset'
      };
    }
  };

  const value: AuthContextType = {
    user,
    isAuthenticated: !!user,
    isLoading,
    login,
    signup,
    logout,
    refreshUser,
    requestEmailVerification,
    resendVerificationEmail,
    requestPasswordReset
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
}

export function useAuth() {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
}

import React, { useState, useRef, useEffect } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import { useHistory } from '@docusaurus/router';
import clsx from 'clsx';
import styles from './UserAuthButton.module.css';

type UserAuthButtonProps = {
  className?: string;
};

export default function UserAuthButton({ className }: UserAuthButtonProps){
  const { user, isAuthenticated, logout, isLoading } = useAuth();
  const [menuOpen, setMenuOpen] = useState(false);
  const menuRef = useRef<HTMLDivElement>(null);
  const history = useHistory();

  // Close menu when clicking outside
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (menuRef.current && !menuRef.current.contains(event.target as Node)) {
        setMenuOpen(false);
      }
    };

    if (menuOpen) {
      document.addEventListener('mousedown', handleClickOutside);
      return () => document.removeEventListener('mousedown', handleClickOutside);
    }
  }, [menuOpen]);

  const handleLogin = () => {
    history.push('/signin');
  };

  const handleSignup = () => {
    history.push('/signup');
  };

  const handleLogout = async () => {
    await logout();
    setMenuOpen(false);
    history.push('/');
  };

  const toggleMenu = () => {
    setMenuOpen(!menuOpen);
  };

  if (isLoading) {
    return null; // or a loading spinner
  }

  return (
    <div className={clsx(styles.authButtonContainer, className)} ref={menuRef}>
      {isAuthenticated && user ? (
        <div className={styles.loggedInContainer}>
          <button
            className={clsx(styles.userButton, styles.loggedIn)}
            onClick={toggleMenu}
            aria-expanded={menuOpen}
            aria-haspopup="true"
          >
            <span className={styles.userInitial}>
              {user.email?.charAt(0).toUpperCase() || 'U'}
            </span>
            <span className={styles.userName}>
              {user.email?.split('@')[0] || 'User'}
            </span>
            <span className={styles.dropdownArrow}>▼</span>
          </button>

          {menuOpen && (
            <div className={styles.dropdownMenu} role="menu">
              <a href="/dashboard" className={styles.menuItem} role="menuitem">
                Dashboard
              </a>
              <a href="/profile" className={styles.menuItem} role="menuitem">
                Profile
              </a>
              <button
                onClick={handleLogout}
                className={clsx(styles.menuItem, styles.logoutButton)}
                role="menuitem"
              >
                Logout
              </button>
            </div>
          )}
        </div>
      ) : (
        <div className={styles.loggedOutContainer}>
          <button
            className={clsx(styles.authButton, styles.loginButton)}
            onClick={handleLogin}
          >
            Sign In
          </button>
          <button
            className={clsx(styles.authButton, styles.signupButton)}
            onClick={handleSignup}
          >
            Sign Up
          </button>
        </div>
      )}
    </div>
  );
}
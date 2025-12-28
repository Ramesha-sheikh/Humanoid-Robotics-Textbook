import React, { useState, useEffect } from 'react';
import { useAuth } from '../Auth/AuthContext';
import { SignupModal } from '../Auth/SignupModal';
import { SigninModal } from '../Auth/SigninModal';
import { ProfileSettingsModal } from '../Auth/ProfileSettingsModal';
import styles from './styles.module.css';

export default function NavbarAuthButton(): JSX.Element {
  const { user, isAuthenticated, signout } = useAuth();
  const [showSignup, setShowSignup] = useState(false);
  const [showSignin, setShowSignin] = useState(false);
  const [showProfile, setShowProfile] = useState(false);
  const [showDropdown, setShowDropdown] = useState(false);

  // Listen for custom events from ProtectedContent
  useEffect(() => {
    const handleOpenSignup = () => setShowSignup(true);
    const handleOpenSignin = () => setShowSignin(true);

    window.addEventListener('openSignupModal', handleOpenSignup);
    window.addEventListener('openSigninModal', handleOpenSignin);

    return () => {
      window.removeEventListener('openSignupModal', handleOpenSignup);
      window.removeEventListener('openSigninModal', handleOpenSignin);
    };
  }, []);

  if (!isAuthenticated) {
    return (
      <>
        <div className={styles.authButtons}>
          <button
            className={styles.signinButton}
            onClick={() => setShowSignin(true)}
            aria-label="Sign In"
          >
            Sign In
          </button>
          <button
            className={styles.signupButton}
            onClick={() => setShowSignup(true)}
            aria-label="Sign Up"
          >
            Sign Up
          </button>
        </div>

        {showSignin && (
          <SigninModal
            onClose={() => setShowSignin(false)}
            onSwitchToSignup={() => {
              setShowSignin(false);
              setShowSignup(true);
            }}
          />
        )}

        {showSignup && (
          <SignupModal
            onClose={() => setShowSignup(false)}
            onSwitchToSignin={() => {
              setShowSignup(false);
              setShowSignin(true);
            }}
          />
        )}
      </>
    );
  }

  return (
    <>
      <div className={styles.profileContainer}>
        <button
          className={styles.profileButton}
          onClick={() => setShowDropdown(!showDropdown)}
          aria-label="User Profile"
          aria-expanded={showDropdown}
        >
          <span className={styles.profileIcon}>👤</span>
          <span className={styles.profileEmail}>{user?.email}</span>
          <span className={styles.dropdownArrow}>▼</span>
        </button>

        {showDropdown && (
          <div className={styles.dropdown}>
            <button
              className={styles.dropdownItem}
              onClick={() => {
                setShowProfile(true);
                setShowDropdown(false);
              }}
            >
              <span className={styles.dropdownIcon}>⚙️</span>
              Profile Settings
            </button>
            <div className={styles.dropdownDivider} />
            <button
              className={styles.dropdownItem}
              onClick={() => {
                signout();
                setShowDropdown(false);
              }}
            >
              <span className={styles.dropdownIcon}>🚪</span>
              Sign Out
            </button>
          </div>
        )}
      </div>

      {showProfile && (
        <ProfileSettingsModal isOpen={showProfile} onClose={() => setShowProfile(false)} />
      )}
    </>
  );
}

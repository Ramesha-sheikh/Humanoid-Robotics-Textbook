import React, { useState, useRef, useEffect } from 'react';
import { useAuth } from './AuthContext';
import { SignupModal } from './SignupModal';
import { SigninModal } from './SigninModal';
import { ProfileSettingsModal } from './ProfileSettingsModal';
import styles from './styles.module.css';

export const AuthButton: React.FC = () => {
  const { user, isAuthenticated, signout } = useAuth();
  const [showSignup, setShowSignup] = useState(false);
  const [showSignin, setShowSignin] = useState(false);
  const [showProfile, setShowProfile] = useState(false);
  const [showDropdown, setShowDropdown] = useState(false);
  const dropdownRef = useRef<HTMLDivElement>(null);

  // Close dropdown when clicking outside
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (dropdownRef.current && !dropdownRef.current.contains(event.target as Node)) {
        setShowDropdown(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, []);

  const handleSignout = () => {
    signout();
    setShowDropdown(false);
  };

  const openProfile = () => {
    setShowProfile(true);
    setShowDropdown(false);
  };

  if (!isAuthenticated) {
    // Anonymous user - show signin/signup button
    return (
      <>
        <button
          className={styles.authButton}
          onClick={() => setShowSignin(true)}
        >
          <span className={styles.authButtonIcon}>🔐</span>
          <span className={styles.authButtonText}>Sign In / Sign Up</span>
        </button>

        <SigninModal
          isOpen={showSignin}
          onClose={() => setShowSignin(false)}
          onSwitchToSignup={() => {
            setShowSignin(false);
            setShowSignup(true);
          }}
        />

        <SignupModal
          isOpen={showSignup}
          onClose={() => setShowSignup(false)}
          onSwitchToSignin={() => {
            setShowSignup(false);
            setShowSignin(true);
          }}
        />
      </>
    );
  }

  // Authenticated user - show email + dropdown
  return (
    <>
      <div className={styles.authButtonContainer} ref={dropdownRef}>
        <button
          className={styles.authButton}
          onClick={() => setShowDropdown(!showDropdown)}
        >
          <span className={styles.authButtonIcon}>👤</span>
          <span className={styles.authButtonText}>{user?.email}</span>
          <span className={styles.dropdownArrow}>▼</span>
        </button>

        {showDropdown && (
          <div className={styles.dropdown}>
            <button className={styles.dropdownItem} onClick={openProfile}>
              <span className={styles.dropdownIcon}>⚙️</span>
              Profile Settings
            </button>
            <div className={styles.dropdownDivider} />
            <button className={styles.dropdownItem} onClick={handleSignout}>
              <span className={styles.dropdownIcon}>🚪</span>
              Sign Out
            </button>
          </div>
        )}
      </div>

      <ProfileSettingsModal
        isOpen={showProfile}
        onClose={() => setShowProfile(false)}
      />
    </>
  );
};

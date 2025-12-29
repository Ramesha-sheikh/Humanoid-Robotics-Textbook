import React, { useState, useEffect } from 'react';
import { createPortal } from 'react-dom';
import { useHistory } from '@docusaurus/router';
import { useAuth } from './AuthContext';
import styles from './styles.module.css';

interface SigninModalProps {
  onClose: () => void;
  onSwitchToSignup: () => void;
}

export const SigninModal: React.FC<SigninModalProps> = ({ onClose, onSwitchToSignup }) => {
  const { signin } = useAuth();
  const history = useHistory();
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [showPassword, setShowPassword] = useState(false);
  const [error, setError] = useState('');
  const [isSubmitting, setIsSubmitting] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setIsSubmitting(true);

    try {
      await signin(email, password);
      // Reset form
      setEmail('');
      setPassword('');
      onClose();
      // Redirect to homepage after successful signin
      history.push('/');
    } catch (err: any) {
      if (err.message.includes('Invalid')) {
        setError('Invalid email or password. Please try again.');
      } else {
        setError('Connection failed. Please try again.');
      }
    } finally {
      setIsSubmitting(false);
    }
  };

  const handleClose = () => {
    setEmail('');
    setPassword('');
    setError('');
    onClose();
  };

  // Render modal at document body level using Portal
  if (typeof document === 'undefined') return null;

  return createPortal(
    <div className={styles.modalOverlay} onClick={handleClose}>
      <div className={styles.modalContent} onClick={(e) => e.stopPropagation()}>
        <button className={styles.closeButton} onClick={handleClose} aria-label="Close">
          ×
        </button>

        <h2 className={styles.modalTitle}>Sign In</h2>
        <p className={styles.modalSubtitle}>Welcome back!</p>

        {error && <div className={styles.errorMessage}>{error}</div>}

        <form onSubmit={handleSubmit} className={styles.authForm}>
          <div className={styles.formGroup}>
            <label htmlFor="signin-email">Email</label>
            <input
              id="signin-email"
              type="email"
              placeholder="your.email@example.com"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              required
              autoComplete="email"
            />
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="signin-password">Password</label>
            <div className={styles.passwordField}>
              <input
                id="signin-password"
                type={showPassword ? 'text' : 'password'}
                placeholder="Enter your password"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                required
                autoComplete="current-password"
              />
              <button
                type="button"
                className={styles.passwordToggle}
                onClick={() => setShowPassword(!showPassword)}
                aria-label={showPassword ? 'Hide password' : 'Show password'}
              >
                {showPassword ? '👁️' : '👁️‍🗨️'}
              </button>
            </div>
          </div>

          <button
            type="submit"
            className={styles.submitButton}
            disabled={isSubmitting}
          >
            {isSubmitting ? 'Signing in...' : 'Sign In'}
          </button>
        </form>

        <p className={styles.switchAuth}>
          Don't have an account?{' '}
          <button onClick={onSwitchToSignup} className={styles.linkButton}>
            Sign up
          </button>
        </p>
      </div>
    </div>,
    document.body
  );
};

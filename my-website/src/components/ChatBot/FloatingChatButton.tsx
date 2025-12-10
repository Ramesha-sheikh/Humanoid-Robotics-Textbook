/**
 * FloatingChatButton Component
 * Floating button to open/close chatbot
 */

import React from 'react';
import styles from './FloatingChatButton.module.css';

export interface FloatingChatButtonProps {
  onClick: () => void;
  isOpen: boolean;
}

export default function FloatingChatButton({
  onClick,
  isOpen,
}: FloatingChatButtonProps): JSX.Element {
  return (
    <button
      className={`${styles.button} ${isOpen ? styles.open : ''}`}
      onClick={onClick}
      aria-label={isOpen ? 'Close chat' : 'Open chat'}
      title={isOpen ? 'Close chat' : 'Ask the textbook chatbot'}
    >
      {isOpen ? (
        <span className={styles.icon}>✖️</span>
      ) : (
        <span className={styles.icon}>💬</span>
      )}
      {!isOpen && <span className={styles.badge}>AI</span>}
    </button>
  );
}

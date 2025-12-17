/**
 * FloatingChatButton Component
 * Floating button to open/close chatbot
 */

import React, { JSX } from 'react';
// import styles from './FloatingChatButton.module.css'; // Removed as we are using global CSS

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
      className={`floating-chat-button ${isOpen ? 'floating-chat-button-open' : ''}`}
      onClick={onClick}
      aria-label={isOpen ? 'Close chat' : 'Open chat'}
      title={isOpen ? 'Close chat' : 'Ask the textbook chatbot'}
    >
      {isOpen ? (
        <span className="floating-chat-icon">✖️</span>
      ) : (
        <span className="floating-chat-icon">💬</span>
      )}
      {!isOpen && <span className="floating-chat-badge">AI</span>}
    </button>
  );
}

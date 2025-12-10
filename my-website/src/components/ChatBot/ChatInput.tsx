/**
 * ChatInput Component
 * Text input with send button for chat messages
 */

import React, { useState, KeyboardEvent } from 'react';
import styles from './ChatInput.module.css';

export interface ChatInputProps {
  onSend: (message: string) => void;
  disabled?: boolean;
  placeholder?: string;
}

export default function ChatInput({
  onSend,
  disabled = false,
  placeholder = 'Ask a question about the textbook...',
}: ChatInputProps): JSX.Element {
  const [input, setInput] = useState('');

  const handleSend = () => {
    const message = input.trim();
    if (message && !disabled) {
      onSend(message);
      setInput('');
    }
  };

  const handleKeyDown = (e: KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  return (
    <div className={styles.inputContainer}>
      <textarea
        className={styles.input}
        value={input}
        onChange={(e) => setInput(e.target.value)}
        onKeyDown={handleKeyDown}
        placeholder={placeholder}
        disabled={disabled}
        rows={1}
        maxLength={500}
      />

      <button
        className={styles.sendButton}
        onClick={handleSend}
        disabled={disabled || !input.trim()}
        aria-label="Send message"
      >
        {disabled ? (
          <span className={styles.spinner}>⏳</span>
        ) : (
          <span>📤</span>
        )}
      </button>
    </div>
  );
}

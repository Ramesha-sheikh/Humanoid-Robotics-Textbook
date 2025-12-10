/**
 * ChatMessage Component
 * Displays individual chat messages (user or assistant)
 */

import React from 'react';
import styles from './ChatMessage.module.css';
import type { Source } from './api';

export interface ChatMessageProps {
  role: 'user' | 'assistant';
  content: string;
  sources?: Source[];
  isStreaming?: boolean;
}

export default function ChatMessage({
  role,
  content,
  sources,
  isStreaming = false,
}: ChatMessageProps): JSX.Element {
  return (
    <div className={`${styles.message} ${styles[role]}`}>
      <div className={styles.avatar}>
        {role === 'user' ? '👤' : '🤖'}
      </div>

      <div className={styles.content}>
        <div className={styles.text}>
          {content}
          {isStreaming && <span className={styles.cursor}>▌</span>}
        </div>

        {sources && sources.length > 0 && (
          <div className={styles.sources}>
            <div className={styles.sourcesLabel}>📚 Sources:</div>
            <div className={styles.sourceChips}>
              {sources.map((source, idx) => (
                <span key={idx} className={styles.sourceChip}>
                  Ch. {source.chapter}, Sec. {source.section}, p.{source.page}
                </span>
              ))}
            </div>
          </div>
        )}
      </div>
    </div>
  );
}

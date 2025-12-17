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
              {sources.map((source, idx) => {
                // Create a readable source string based on available properties
                const sourceParts = [];
                if (source.chapter) sourceParts.push(`Ch. ${source.chapter}`);
                if (source.section) sourceParts.push(`Sec. ${source.section}`);
                if (source.page) sourceParts.push(`p.${source.page}`);
                if (source.url) sourceParts.push(source.url);

                const sourceText = sourceParts.length > 0 ? sourceParts.join(', ') : 'Source';
                return (
                  <span key={idx} className={styles.sourceChip}>
                    {sourceText}
                  </span>
                );
              })}
            </div>
          </div>
        )}
      </div>
    </div>
  );
}

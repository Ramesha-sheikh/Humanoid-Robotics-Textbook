/**
 * ChatMessage Component
 * Displays individual chat messages (user or assistant)
 */

import React from 'react';
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
    <div className={`chat-message ${role}`}>
      <div className="chat-avatar">
        {role === 'user' ? '👤' : '🤖'}
      </div>

      <div className="chat-content">
        <div className="chat-text">
          {content}
          {isStreaming && <span className="chat-cursor">▌</span>}
        </div>

        {sources && sources.length > 0 && (
          <div className="chat-sources">
            <div className="chat-sources-label">📚 Sources:</div>
            <div className="chat-source-chips">
              {sources.map((source, idx) => {
                // Create a readable source string based on available properties
                const sourceParts = [];
                if (source.chapter) sourceParts.push(`Ch. ${source.chapter}`);
                if (source.section) sourceParts.push(`Sec. ${source.section}`);
                if (source.page) sourceParts.push(`p.${source.page}`);
                if (source.url) sourceParts.push(source.url);

                const sourceText = sourceParts.length > 0 ? sourceParts.join(', ') : 'Source';
                return (
                  <span key={idx} className="chat-source-chip">
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

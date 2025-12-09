import React from 'react';
import clsx from 'clsx'; // A utility for constructing className strings conditionally
import styles from './ChatMessage.module.css'; // We'll create this CSS module

interface Source {
  title: string;
  url: string;
  heading?: string;
}

interface ChatMessageProps {
  message: string;
  isUser: boolean;
  sources?: Source[];
}

const ChatMessage: React.FC<ChatMessageProps> = ({ message, isUser, sources }) => {
  return (
    <div className={clsx(styles.messageContainer, isUser ? styles.userMessage : styles.botMessage)}>
      <div className={styles.messageContent}>
        {message}
        {!isUser && sources && sources.length > 0 && (
          <div className={styles.sourceChips}>
            {sources.map((source, index) => (
              <a
                key={index}
                href={source.url}
                target="_blank"
                rel="noopener noreferrer"
                className={styles.sourceChip}
                title={source.heading ? `${source.title} - ${source.heading}` : source.title}
              >
                {source.title}
              </a>
            ))}
          </div>
        )}
      </div>
    </div>
  );
};

export default ChatMessage;
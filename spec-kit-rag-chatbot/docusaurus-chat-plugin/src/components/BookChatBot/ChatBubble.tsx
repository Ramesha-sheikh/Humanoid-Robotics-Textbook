import React, { useState } from 'react';
import ChatWindow from './ChatWindow'; // Import the ChatWindow component
import styles from './ChatBubble.module.css';

const ChatBubble: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  return (
    <div className={styles.chatBubbleContainer}>
      {isOpen ? (
        <div className={styles.chatWindowWrapper}>
          <ChatWindow />
          <button className={styles.closeChatButton} onClick={toggleChat}>
            &times; {/* Close icon */}
          </button>
        </div>
      ) : (
        <button className={styles.chatBubbleButton} onClick={toggleChat}>
          Chat
        </button>
      )}
    </div>
  );
};

export default ChatBubble;
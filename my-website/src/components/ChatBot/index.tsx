/**
 * ChatBot Component
 * Main export with floating button and chat window
 */

import React, { JSX, useState } from 'react';
import ChatWindow from './ChatWindow';
import FloatingChatButton from './FloatingChatButton';

export default function ChatBot(): JSX.Element {
  const [isOpen, setIsOpen] = useState(false);

  const toggleChat = () => {
    setIsOpen((prev) => !prev);
  };

  return (
    <>
      <FloatingChatButton onClick={toggleChat} isOpen={isOpen} />
      <ChatWindow isOpen={isOpen} onClose={() => setIsOpen(false)} />
    </>
  );
}

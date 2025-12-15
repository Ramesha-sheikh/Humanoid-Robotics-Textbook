import React, { useState } from 'react';
import FloatingChatButton from './ChatBot/FloatingChatButton';
import ChatWindow from './ChatBot/ChatWindow';

const RagChatbot = () => {
  const [isOpen, setIsOpen] = useState(false);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  return (
    <>
      <FloatingChatButton onClick={toggleChat} isOpen={isOpen} />
      {isOpen && <ChatWindow onClose={toggleChat} isOpen={isOpen} />}
    </>
  );
};

export default RagChatbot;

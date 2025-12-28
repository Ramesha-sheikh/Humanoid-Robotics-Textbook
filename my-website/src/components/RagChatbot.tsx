import React, { useState } from 'react';
import FloatingChatButton from './ChatBot/FloatingChatButton';
import ChatWindow from './ChatBot/ChatWindow';
import { useAuth } from './Auth/AuthContext';

const RagChatbot = () => {
  const { isAuthenticated } = useAuth();
  const [isOpen, setIsOpen] = useState(false);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  // Only show chatbot if user is authenticated
  if (!isAuthenticated) {
    return null;
  }

  return (
    <>
      <FloatingChatButton onClick={toggleChat} isOpen={isOpen} />
      {isOpen && <ChatWindow onClose={toggleChat} isOpen={isOpen} />}
    </>
  );
};

export default RagChatbot;

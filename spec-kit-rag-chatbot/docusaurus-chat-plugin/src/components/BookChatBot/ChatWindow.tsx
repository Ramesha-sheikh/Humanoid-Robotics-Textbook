import React, { useState, useEffect, useRef } from 'react';
import ChatMessage from './ChatMessage';
import ChatInput from './ChatInput';
import { streamChatResponse, ChatMessageData } from './api';
import styles from './ChatWindow.module.css'; // We'll create this CSS module
import { v4 as uuidv4 } from 'uuid'; // For generating unique session/user IDs

// Mock UUID for now, replace with actual import if needed
const mockUuidv4 = () => Math.random().toString(36).substring(2, 15);

interface Message {
  id: string;
  text: string;
  isUser: boolean;
  sources?: Array<{ title: string; url: string; heading?: string }>;
}

const ChatWindow: React.FC = () => {
  const [messages, setMessages] = useState<Message[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState<string>(mockUuidv4()); // Use UUID for session
  const chatMessagesRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    // Scroll to bottom on new message
    if (chatMessagesRef.current) {
      chatMessagesRef.current.scrollTop = chatMessagesRef.current.scrollHeight;
    }
  }, [messages]);

  const handleSendMessage = async (userQuery: string) => {
    const newUserMessage: Message = { id: mockUuidv4(), text: userQuery, isUser: true };
    setMessages((prevMessages) => [...prevMessages, newUserMessage]);
    setIsLoading(true);

    let botResponseContent = '';
    let botResponseSources: Array<{ title: string; url: string; heading?: string }> = [];

    const handleStreamMessage = (data: ChatMessageData) => {
      if (data.type === 'text' && data.content) {
        botResponseContent += data.content;
        setMessages((prevMessages) => {
          // Find the last bot message and update it
          const lastBotMessageIndex = prevMessages.findIndex(
            (msg) => !msg.isUser && msg.id === 'streaming-bot-message'
          );
          if (lastBotMessageIndex !== -1) {
            const updatedMessages = [...prevMessages];
            updatedMessages[lastBotMessageIndex] = {
              ...updatedMessages[lastBotMessageIndex],
              text: botResponseContent,
            };
            return updatedMessages;
          } else {
            // Add a new message if it's the first chunk
            return [
              ...prevMessages,
              { id: 'streaming-bot-message', text: botResponseContent, isUser: false },
            ];
          }
        });
      } else if (data.type === 'final' && data.answer) {
        botResponseContent = data.answer;
        botResponseSources = data.sources || [];
        setSessionId(data.session_id || sessionId); // Update session_id if provided by backend

        setMessages((prevMessages) => {
          // Replace the streaming bot message with the final one
          const updatedMessages = prevMessages.map((msg) =>
            msg.id === 'streaming-bot-message'
              ? { ...msg, text: botResponseContent, sources: botResponseSources }
              : msg
          );
          return updatedMessages;
        });
        setIsLoading(false);
      }
    };

    const handleStreamError = (error: Event) => {
      console.error('Chat stream error:', error);
      setMessages((prevMessages) => [
        ...prevMessages,
        {
          id: mockUuidv4(),
          text: 'Sorry, something went wrong. Please try again.',
          isUser: false,
        },
      ]);
      setIsLoading(false);
    };

    try {
      await streamChatResponse(userQuery, handleStreamMessage, handleStreamError, sessionId);
    } catch (error) {
      console.error('Initial stream setup error:', error);
      handleStreamError(error as Event);
    }
  };

  return (
    <div className={styles.chatWindowContainer}>
      <div className={styles.chatHeader}>Book Chatbot</div>
      <div ref={chatMessagesRef} className={styles.chatMessages}>
        {messages.map((msg) => (
          <ChatMessage key={msg.id} message={msg.text} isUser={msg.isUser} sources={msg.sources} />
        ))}
        {isLoading && (
          <ChatMessage id="loading-indicator" message="Typing..." isUser={false} />
        )}
      </div>
      <ChatInput onSendMessage={handleSendMessage} isLoading={isLoading} />
    </div>
  );
};

export default ChatWindow;
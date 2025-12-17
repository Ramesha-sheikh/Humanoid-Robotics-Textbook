/**
 * ChatWindow Component
 * Main chatbot interface with message history and streaming
 */

import React, { useState, useEffect, useRef } from 'react';
import ChatMessage from './ChatMessage';
import ChatInput from './ChatInput';
import { streamChatMessage, type ChatResponse } from './api';
// import styles from './ChatWindow.module.css'; // Removed as we are using global CSS

export interface Message {
  role: 'user' | 'assistant';
  content: string;
  sources?: string[];
  isStreaming?: boolean;
}

export interface ChatWindowProps {
  isOpen: boolean;
  onClose: () => void;
  // New prop for selected text context
  selectedTextContext?: string;
  onSendWithContext?: (question: string, context: string) => void;
}

export default function ChatWindow({ isOpen, onClose, selectedTextContext, onSendWithContext }: ChatWindowProps): JSX.Element | null {
  const [messages, setMessages] = useState<Message[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  // const [mode, setMode] = useState<'normal' | 'highlight'>('normal'); // Removed mode state
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Welcome message on first load
  useEffect(() => {
    if (messages.length === 0) {
      setMessages([
        {
          role: 'assistant',
          content: 'Hello! I\'m your Humanoid Robotics textbook assistant. Ask me anything about the book! 🤖📚',
        },
      ]);
    }
  }, []);

  const handleSendMessage = async (question: string, selectedText?: string) => {
    if (!question.trim() || isLoading) return;

    // Add user message
    const userMessage: Message = { role: 'user', content: question };
    setMessages((prev) => [...prev, userMessage]);

    // Start streaming assistant response
    setIsLoading(true);

    const assistantMessage: Message = {
      role: 'assistant',
      content: '',
      isStreaming: true,
    };

    setMessages((prev) => [...prev, assistantMessage]);

    try {
      // Determine if there's selected text from context menu or direct input
      const textToSend = selectedTextContext || selectedText;

      // Stream response tokens
      for await (const chunk of streamChatMessage(question, textToSend)) {
        if (chunk.error) {
          assistantMessage.content += `\n\n❌ Error: ${chunk.error}`;
          assistantMessage.isStreaming = false;
          setMessages((prev) => [...prev.slice(0, -1), { ...assistantMessage }]);
          break;
        }

        if (chunk.done) {
          if (chunk.response) {
            // Use the response from the final chunk instead of accumulated tokens
            assistantMessage.content = chunk.response.answer;

            // Convert string sources to Source objects if needed
            if (chunk.response.sources) {
              // Check if sources are already in the correct format (Source objects)
              if (chunk.response.sources.length > 0 && typeof chunk.response.sources[0] === 'string') {
                // Sources are strings, convert to Source objects with URL
                assistantMessage.sources = (chunk.response.sources as string[]).map(url => ({ url }));
              } else {
                // Sources are already Source objects
                assistantMessage.sources = chunk.response.sources as any;
              }
            }
          }
          assistantMessage.isStreaming = false;
          setMessages((prev) => [...prev.slice(0, -1), { ...assistantMessage }]);
          break;
        }

        // Append token to message
        if (chunk.token) {
          assistantMessage.content += chunk.token;
          setMessages((prev) => [...prev.slice(0, -1), { ...assistantMessage }]);
        }
      }
    } catch (error) {
      assistantMessage.content = `❌ Connection error: ${error.message}. Make sure the backend is running at http://localhost:8001`;
      assistantMessage.isStreaming = false;
      setMessages((prev) => [...prev.slice(0, -1), { ...assistantMessage }]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleClearChat = () => {
    setMessages([
      {
        role: 'assistant',
        content: 'Chat cleared! Ask me anything about the textbook. 🤖',
      },
    ]);
  };

  if (!isOpen) {
    console.log('ChatWindow: Not open, returning null.');
    return null;
  }
  console.log('ChatWindow: isOpen is true, attempting to render.');

  return (
    // Use global CSS class
    <div className="chat-window-container" onClick={(e) => e.stopPropagation()}>
      {/* Header */}
      <div className="chat-window-header">
        <h3>🤖 RAG Chatbot</h3>
        {/* Removed mode badge */}
        <button
          className="chat-window-close-button"
          onClick={onClose}
          title="Close chat"
        >
          ✖️
        </button>
      </div>

      {/* Messages */}
      <div className="chat-messages">
        {messages.map((msg, idx) => (
          <ChatMessage
            key={idx}
            role={msg.role}
            content={msg.content}
            sources={msg.sources}
            isStreaming={msg.isStreaming}
          />
        ))}
        <div ref={messagesEndRef} />
      </div>

      {/* Input */}
      <ChatInput
        onSend={(question) => handleSendMessage(question, selectedTextContext)} // Pass selectedTextContext
        disabled={isLoading}
        placeholder={isLoading ? 'Generating response...' : (selectedTextContext ? 'Ask about highlighted text...' : 'Ask a question about the textbook...')}
      />

      {/* Footer */}
      <div className="chat-footer">
        <span className="chat-footer-text">
          Powered by Cohere Command R+ ⚡
        </span>
      </div>
    </div>
  );
}

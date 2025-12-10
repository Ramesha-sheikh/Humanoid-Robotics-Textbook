/**
 * ChatWindow Component
 * Main chatbot interface with message history and streaming
 */

import React, { useState, useEffect, useRef } from 'react';
import ChatMessage from './ChatMessage';
import ChatInput from './ChatInput';
import { streamChatMessage, type ChatRequest, type Source } from './api';
import styles from './ChatWindow.module.css';

export interface Message {
  role: 'user' | 'assistant';
  content: string;
  sources?: Source[];
  isStreaming?: boolean;
}

export interface ChatWindowProps {
  isOpen: boolean;
  onClose: () => void;
}

export default function ChatWindow({ isOpen, onClose }: ChatWindowProps): JSX.Element | null {
  const [messages, setMessages] = useState<Message[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [mode, setMode] = useState<'normal' | 'highlight'>('normal');
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

  const handleSendMessage = async (query: string) => {
    if (!query.trim() || isLoading) return;

    // Add user message
    const userMessage: Message = { role: 'user', content: query };
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
      const request: ChatRequest = {
        query,
        mode,
        // TODO: Add highlight_context when implementing highlight mode
      };

      // Stream response tokens
      for await (const chunk of streamChatMessage(request)) {
        if (chunk.error) {
          assistantMessage.content += `\n\n❌ Error: ${chunk.error}`;
          assistantMessage.isStreaming = false;
          setMessages((prev) => [...prev.slice(0, -1), { ...assistantMessage }]);
          break;
        }

        if (chunk.done) {
          assistantMessage.isStreaming = false;
          // TODO: Extract sources from response
          setMessages((prev) => [...prev.slice(0, -1), { ...assistantMessage }]);
          break;
        }

        // Append token to message
        assistantMessage.content += chunk.token;
        setMessages((prev) => [...prev.slice(0, -1), { ...assistantMessage }]);
      }
    } catch (error) {
      assistantMessage.content = `❌ Connection error: ${error.message}. Make sure the backend is running at http://localhost:8000`;
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

  if (!isOpen) return null;

  return (
    <div className={styles.overlay} onClick={onClose}>
      <div className={styles.window} onClick={(e) => e.stopPropagation()}>
        {/* Header */}
        <div className={styles.header}>
          <div className={styles.headerLeft}>
            <h3 className={styles.title}>🤖 RAG Chatbot</h3>
            <span className={styles.badge}>{mode === 'normal' ? '📚 Normal' : '✨ Highlight'}</span>
          </div>
          <div className={styles.headerRight}>
            <button
              className={styles.clearButton}
              onClick={handleClearChat}
              title="Clear chat"
            >
              🗑️
            </button>
            <button
              className={styles.closeButton}
              onClick={onClose}
              title="Close chat"
            >
              ✖️
            </button>
          </div>
        </div>

        {/* Messages */}
        <div className={styles.messages}>
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
          onSend={handleSendMessage}
          disabled={isLoading}
          placeholder={
            isLoading
              ? 'Generating response...'
              : mode === 'highlight'
              ? 'Ask about highlighted text...'
              : 'Ask a question about the textbook...'
          }
        />

        {/* Footer */}
        <div className={styles.footer}>
          <span className={styles.footerText}>
            Powered by Gemini 2.0 Flash ⚡
          </span>
        </div>
      </div>
    </div>
  );
}

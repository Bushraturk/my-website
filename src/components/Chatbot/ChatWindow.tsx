import React, { useEffect, useRef } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import Translate from '@docusaurus/Translate';
import { useChatContext } from '../../contexts/ChatContext';
import ChatHistory from './ChatHistory';
import ChatInput from './ChatInput';
import styles from './ChatWindow.module.css';

const ChatWindowContent: React.FC = () => {
  const { isOpen, toggleChat, clearHistory, messages } = useChatContext();
  const windowRef = useRef<HTMLDivElement>(null);

  // Handle escape key to close
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape' && isOpen) {
        toggleChat();
      }
    };

    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, [isOpen, toggleChat]);

  // Focus management
  useEffect(() => {
    if (isOpen && windowRef.current) {
      const firstInput = windowRef.current.querySelector('textarea');
      firstInput?.focus();
    }
  }, [isOpen]);

  if (!isOpen) return null;

  return (
    <div
      ref={windowRef}
      className={styles.chatWindow}
      role="dialog"
      aria-labelledby="chat-title"
      aria-modal="true"
    >
      {/* Header */}
      <div className={styles.header}>
        <div className={styles.titleSection}>
          <span className={styles.robotEmoji}>🤖</span>
          <h2 id="chat-title" className={styles.title}>
            <Translate id="chatbot.title">AI Assistant</Translate>
          </h2>
        </div>
        <div className={styles.controls}>
          {messages.length > 0 && (
            <button
              className={styles.controlButton}
              onClick={clearHistory}
              aria-label="Clear chat history"
              title="Clear history"
            >
              🗑️
            </button>
          )}
          <button
            className={styles.controlButton}
            onClick={toggleChat}
            aria-label="Close chat"
            title="Close"
          >
            ✕
          </button>
        </div>
      </div>

      {/* Chat History */}
      <ChatHistory />

      {/* Input */}
      <ChatInput />
    </div>
  );
};

const ChatWindow: React.FC = () => {
  return (
    <BrowserOnly fallback={null}>
      {() => <ChatWindowContent />}
    </BrowserOnly>
  );
};

export default ChatWindow;

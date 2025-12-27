import React, { useEffect, useRef } from 'react';
import Translate from '@docusaurus/Translate';
import { useChatContext } from '../../contexts/ChatContext';
import ChatMessage from './ChatMessage';
import styles from './ChatHistory.module.css';

const ChatHistory: React.FC = () => {
  const { messages, isLoading } = useChatContext();
  const containerRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom on new messages
  useEffect(() => {
    if (containerRef.current) {
      containerRef.current.scrollTop = containerRef.current.scrollHeight;
    }
  }, [messages, isLoading]);

  return (
    <div
      ref={containerRef}
      className={styles.historyContainer}
      role="log"
      aria-live="polite"
      aria-atomic="false"
    >
      {messages.length === 0 ? (
        <div className={styles.emptyState}>
          <div className={styles.emptyIcon}>🤖</div>
          <h3 className={styles.emptyTitle}>
            <Translate id="chatbot.empty.title">Welcome!</Translate>
          </h3>
          <p className={styles.emptyText}>
            <Translate id="chatbot.empty">
              Start a conversation! Ask me anything about Physical AI and Robotics.
            </Translate>
          </p>
          <div className={styles.suggestions}>
            <p className={styles.suggestionsTitle}>Try asking:</p>
            <ul className={styles.suggestionsList}>
              <li>What is ROS 2?</li>
              <li>How does NVIDIA Isaac work?</li>
              <li>Explain Vision-Language-Action models</li>
            </ul>
          </div>
        </div>
      ) : (
        <>
          {messages.map((message) => (
            <ChatMessage key={message.id} message={message} />
          ))}
          {isLoading && (
            <div className={styles.loadingContainer}>
              <div className={styles.avatar}>🤖</div>
              <div className={styles.typingIndicator}>
                <span></span>
                <span></span>
                <span></span>
              </div>
            </div>
          )}
        </>
      )}
    </div>
  );
};

export default ChatHistory;

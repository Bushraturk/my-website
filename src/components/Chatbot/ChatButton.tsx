import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { useChatContext } from '../../contexts/ChatContext';
import styles from './ChatButton.module.css';

const ChatButtonContent: React.FC = () => {
  const { isOpen, toggleChat, unreadCount } = useChatContext();

  return (
    <button
      className={`${styles.chatButton} ${isOpen ? styles.active : ''}`}
      onClick={toggleChat}
      aria-label={isOpen ? 'Close AI Chat' : 'Open AI Chat'}
      aria-expanded={isOpen}
    >
      <span className={styles.robotIcon}>
        {isOpen ? '✕' : '🤖'}
      </span>
      {!isOpen && unreadCount > 0 && (
        <span className={styles.badge} aria-label={`${unreadCount} unread messages`}>
          {unreadCount > 9 ? '9+' : unreadCount}
        </span>
      )}
    </button>
  );
};

const ChatButton: React.FC = () => {
  return (
    <BrowserOnly fallback={null}>
      {() => <ChatButtonContent />}
    </BrowserOnly>
  );
};

export default ChatButton;

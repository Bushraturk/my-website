import React from 'react';
import type { ChatMessage as ChatMessageType } from '../../types/chat';
import styles from './ChatMessage.module.css';

interface ChatMessageProps {
  message: ChatMessageType;
}

const formatTime = (date: Date): string => {
  return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
};

const ChatMessage: React.FC<ChatMessageProps> = ({ message }) => {
  const isUser = message.role === 'user';
  const isError = message.error;

  return (
    <div
      className={`${styles.messageContainer} ${isUser ? styles.user : styles.assistant}`}
    >
      {/* Avatar */}
      <div className={styles.avatar}>
        {isUser ? '👤' : '🤖'}
      </div>

      {/* Message Content */}
      <div
        className={`${styles.messageBubble} ${isError ? styles.error : ''}`}
      >
        <div className={styles.content}>
          {message.content}
        </div>
        <div className={styles.timestamp}>
          {formatTime(new Date(message.timestamp))}
        </div>
      </div>
    </div>
  );
};

export default ChatMessage;

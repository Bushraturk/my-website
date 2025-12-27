import React, { createContext, useContext, useState, useEffect, useCallback } from 'react';
import type { ChatMessage, ChatContextType, UserBackground } from '../types/chat';
import { sendMessage as sendApiMessage } from '../services/chatApi';

const defaultContext: ChatContextType = {
  messages: [],
  isOpen: false,
  isLoading: false,
  error: null,
  unreadCount: 0,
  sendMessage: async () => {},
  toggleChat: () => {},
  clearHistory: () => {},
  markAsRead: () => {},
};

const ChatContext = createContext<ChatContextType>(defaultContext);

export const useChatContext = () => {
  const context = useContext(ChatContext);
  return context;
};

interface ChatProviderProps {
  children: React.ReactNode;
}

const generateId = () => `${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;

const getStorageKey = (userId?: string) => `chat_history_${userId || 'guest'}`;

export const ChatProvider: React.FC<ChatProviderProps> = ({ children }) => {
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [isOpen, setIsOpen] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [unreadCount, setUnreadCount] = useState(0);
  const [userBackground, setUserBackground] = useState<UserBackground | null>(null);
  const [userId, setUserId] = useState<string | undefined>(undefined);

  // Load user data and chat history from localStorage
  useEffect(() => {
    if (typeof window !== 'undefined') {
      try {
        // Try to get user data from localStorage
        const userData = localStorage.getItem('user');
        if (userData) {
          const user = JSON.parse(userData);
          setUserId(user.id);
          if (user.background) {
            setUserBackground(user.background);
          }
        }

        // Load chat history
        const storageKey = getStorageKey(userId);
        const savedMessages = localStorage.getItem(storageKey);
        if (savedMessages) {
          const parsed = JSON.parse(savedMessages);
          setMessages(parsed.map((msg: ChatMessage) => ({
            ...msg,
            timestamp: new Date(msg.timestamp),
          })));
        }
      } catch (e) {
        console.error('Failed to load chat data:', e);
      }
    }
  }, [userId]);

  // Save messages to localStorage
  useEffect(() => {
    if (typeof window !== 'undefined' && messages.length > 0) {
      try {
        const storageKey = getStorageKey(userId);
        localStorage.setItem(storageKey, JSON.stringify(messages));
      } catch (e) {
        console.error('Failed to save chat history:', e);
      }
    }
  }, [messages, userId]);

  // Reset unread count when chat is opened
  useEffect(() => {
    if (isOpen) {
      setUnreadCount(0);
    }
  }, [isOpen]);

  const toggleChat = useCallback(() => {
    setIsOpen(prev => !prev);
    setError(null);
  }, []);

  const clearHistory = useCallback(() => {
    setMessages([]);
    if (typeof window !== 'undefined') {
      const storageKey = getStorageKey(userId);
      localStorage.removeItem(storageKey);
    }
  }, [userId]);

  const markAsRead = useCallback(() => {
    setUnreadCount(0);
  }, []);

  const sendMessage = useCallback(async (content: string) => {
    if (!content.trim() || isLoading) return;

    // Add user message
    const userMessage: ChatMessage = {
      id: generateId(),
      role: 'user',
      content: content.trim(),
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);
    setError(null);

    try {
      const response = await sendApiMessage(content, userBackground, messages);

      // Add assistant message
      const assistantMessage: ChatMessage = {
        id: generateId(),
        role: 'assistant',
        content: response.response,
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, assistantMessage]);

      // Increment unread if chat is closed
      if (!isOpen) {
        setUnreadCount(prev => prev + 1);
      }
    } catch (e) {
      const errorMessage = e instanceof Error ? e.message : 'Failed to send message';
      setError(errorMessage);

      // Add error message
      const errorChatMessage: ChatMessage = {
        id: generateId(),
        role: 'assistant',
        content: errorMessage,
        timestamp: new Date(),
        error: true,
      };

      setMessages(prev => [...prev, errorChatMessage]);
    } finally {
      setIsLoading(false);
    }
  }, [isLoading, userBackground, messages, isOpen]);

  const value: ChatContextType = {
    messages,
    isOpen,
    isLoading,
    error,
    unreadCount,
    sendMessage,
    toggleChat,
    clearHistory,
    markAsRead,
  };

  return (
    <ChatContext.Provider value={value}>
      {children}
    </ChatContext.Provider>
  );
};

export default ChatProvider;

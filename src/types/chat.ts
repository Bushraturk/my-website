export interface ChatMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  isStreaming?: boolean;
  error?: boolean;
}

export interface ChatState {
  messages: ChatMessage[];
  isOpen: boolean;
  isLoading: boolean;
  error: string | null;
  unreadCount: number;
}

export interface ChatContextType extends ChatState {
  sendMessage: (content: string) => Promise<void>;
  toggleChat: () => void;
  clearHistory: () => void;
  markAsRead: () => void;
}

export interface UserBackground {
  programmingExperience: 'beginner' | 'intermediate' | 'advanced' | null;
  hardwareExperience: 'none' | 'basic' | 'intermediate' | 'advanced' | null;
  roboticsBackground: 'none' | 'academic' | 'industry' | null;
  interests: string[];
}

export interface ChatRequest {
  message: string;
  user_context: {
    programming_experience: string;
    hardware_experience: string;
    robotics_background: string;
    interests: string[];
  };
  conversation_history?: Array<{
    role: string;
    content: string;
  }>;
}

export interface ChatResponse {
  response: string;
  metadata?: {
    sources?: string[];
    confidence?: number;
  };
}

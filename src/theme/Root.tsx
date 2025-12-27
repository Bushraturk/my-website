import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { ChatProvider } from '../contexts/ChatContext';
import { ChatButton, ChatWindow } from '../components/Chatbot';

// Root component with ChatProvider for global chatbot access
const Root = ({ children }) => {
  return (
    <ChatProvider>
      {children}
      <BrowserOnly fallback={null}>
        {() => (
          <>
            <ChatButton />
            <ChatWindow />
          </>
        )}
      </BrowserOnly>
    </ChatProvider>
  );
};

export default Root;

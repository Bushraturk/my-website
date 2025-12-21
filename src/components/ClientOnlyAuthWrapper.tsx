import React from 'react';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import { AuthProvider } from '../contexts/AuthContext';
import { LanguageProvider } from '../contexts/LanguageContext';

// This component ensures auth providers are only rendered on the client
const ClientOnlyAuthWrapper: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  // Only render providers in browser environment to avoid SSR issues
  if (ExecutionEnvironment.canUseDOM) {
    return (
      <LanguageProvider>
        <AuthProvider>
          {children}
        </AuthProvider>
      </LanguageProvider>
    );
  }

  // During SSR, just return children without providers
  return <>{children}</>;
};

export default ClientOnlyAuthWrapper;
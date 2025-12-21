// __tests__/test-utils.tsx
// Test utilities for the Physical AI & Humanoid Robotics textbook

import React, { ReactElement } from 'react';
import { render, RenderOptions } from '@testing-library/react';
import { AuthProvider } from '../src/contexts/AuthContext';
import { LanguageProvider } from '../src/contexts/LanguageContext';

// Wrapper component that includes all necessary providers for testing
const AllProviders = ({ children }: { children: React.ReactNode }) => {
  return (
    <LanguageProvider>
      <AuthProvider>
        {children}
      </AuthProvider>
    </LanguageProvider>
  );
};

// Custom render function that includes providers
const customRender = (
  ui: ReactElement,
  options?: Omit<RenderOptions, 'wrapper'>
) => render(ui, { wrapper: AllProviders, ...options });

// Export everything from testing-library/react
export * from '@testing-library/react';

// Override render method to include providers
export { customRender as render };
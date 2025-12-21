import React, { createContext, useContext, useState, ReactNode, useEffect, useCallback } from 'react';

// Define language types
type Language = 'en' | 'ur';

// Simple translations dictionary (add more as needed)
const translations: Record<string, Record<string, string>> = {
  ur: {
    // Add your Urdu translations here
    'Hello': 'ہیلو',
    'Welcome': 'خوش آمدید',
    'Difficulty': 'مشکل',
    'Duration': 'دورانیہ',
    'Progress': 'پیش رفت',
    'Start Learning': 'سیکھنا شروع کریں',
    'Learning Objectives': 'سیکھنے کے مقاصد',
    'Prerequisites': 'لازمی شرائط',
    'Lab Content': 'لیب مواد',
    'Assessment Submitted': 'تشخیص جمع کرائی گئی',
    'Thank you for completing the assessment.': 'تشخیص مکمل کرنے کا شکریہ۔',
    'Previous': 'پچھلا',
    'Next': 'اگلا',
    'Submit': 'جمع کرائیں',
  }
};

// Define the context type
interface LanguageContextType {
  language: Language;
  toggleLanguage: () => void;
  translate: (text: string) => string;
}

// Default context value for SSR safety
const defaultLanguageContext: LanguageContextType = {
  language: 'en',
  toggleLanguage: () => {},
  translate: (text: string) => text,
};

// Create the context with default value
const LanguageContext = createContext<LanguageContextType>(defaultLanguageContext);

// Provider component
interface LanguageProviderProps {
  children: ReactNode;
}

export const LanguageProvider: React.FC<LanguageProviderProps> = ({ children }) => {
  const [language, setLanguage] = useState<Language>('en');

  // Initialize language from localStorage when component mounts
  useEffect(() => {
    if (typeof window !== 'undefined') {
      const savedLanguage = localStorage.getItem('preferredLanguage') as Language | null;
      if (savedLanguage && ['en', 'ur'].includes(savedLanguage)) {
        setLanguage(savedLanguage);
      }
    }
  }, []);

  // Save language preference to localStorage
  useEffect(() => {
    if (typeof window !== 'undefined') {
      localStorage.setItem('preferredLanguage', language);
    }
  }, [language]);

  const toggleLanguage = () => {
    setLanguage(prev => prev === 'en' ? 'ur' : 'en');
  };

  // Translate function - returns translated text or original if not found
  const translate = useCallback((text: string): string => {
    if (language === 'en') {
      return text;
    }
    return translations[language]?.[text] || text;
  }, [language]);

  return (
    <LanguageContext.Provider value={{ language, toggleLanguage, translate }}>
      {children}
    </LanguageContext.Provider>
  );
};

// Custom hook to use the language context - now safe for SSR
export const useLanguage = (): LanguageContextType => {
  const context = useContext(LanguageContext);
  // No longer throws - returns default context if not wrapped in provider
  return context;
};

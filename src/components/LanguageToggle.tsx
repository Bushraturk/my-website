import React from 'react';
import { useLanguage } from '../contexts/LanguageContext';

const LanguageToggle: React.FC = () => {
  const { language, toggleLanguage } = useLanguage();

  return (
    <button
      onClick={toggleLanguage}
      className="language-toggle-button"
      style={{
        backgroundColor: language === 'ur' ? '#FFD700' : '#333', // Gold when Urdu is active, dark when English is active
        color: language === 'ur' ? '#000' : '#FFF', // Black text when Urdu is active, white when English
        border: '1px solid #FFD700', // Gold border
        borderRadius: '4px',
        padding: '0.5rem 1rem',
        margin: '0 1rem',
        cursor: 'pointer',
        fontWeight: 'bold',
        fontSize: '0.9rem',
        transition: 'all 0.3s ease',
      }}
      aria-label={`Switch language to ${language === 'en' ? 'Urdu' : 'English'}`}
    >
      {language === 'en' ? 'UR' : 'EN'}
    </button>
  );
};

export default LanguageToggle;
import React from 'react';
import { useLanguage } from '../contexts/LanguageContext';

interface TranslateProps {
  children: string;
}

const Translate: React.FC<TranslateProps> = ({ children }) => {
  const { translate } = useLanguage();
  
  return <>{translate(children)}</>;
};

export default Translate;
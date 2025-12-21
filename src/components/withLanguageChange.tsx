import React, { ComponentType, useEffect, useState } from 'react';
import { useLanguage } from '../contexts/LanguageContext';

/**
 * Higher-Order Component that re-renders wrapped content when language changes
 */
const withLanguageChange = <P extends object>(WrappedComponent: ComponentType<P>) => {
  const WithLanguageChange = (props: P) => {
    const { language } = useLanguage();
    const [version, setVersion] = useState(0);
    
    // Update version when language changes to force re-render
    useEffect(() => {
      setVersion(prev => prev + 1);
    }, [language]);

    // Return the wrapped component with a new key to force re-render
    return <WrappedComponent key={`lang-${version}`} {...props} />;
  };

  return WithLanguageChange;
};

export default withLanguageChange;
import { useEffect } from 'react';
import { useLanguage } from '../contexts/LanguageContext';
import { useLocation } from '@docusaurus/router';

/**
 * DOM-based translation component that directly manipulates content
 * This approach is more reliable for Docusaurus static content
 * Uses efficient DOM queries to find specifically marked elements
 */
const DOMTranslateHandler = () => {
  const { language, translate } = useLanguage();
  const location = useLocation(); // Track route changes

  useEffect(() => {
    // Function to translate marked elements specifically
    const translateDocument = () => {
      // Find specifically marked elements with our translation attribute
      const elementsToTranslate = document.querySelectorAll('[data-docusaurus-translate="true"]');

      elementsToTranslate.forEach(element => {
        // Get text content from the data-original attribute since DOMTranslate stores it there
        const originalText = element.getAttribute('data-original') || element.textContent.trim();
        const currentText = element.textContent.trim();

        if (language === 'ur' && originalText) {
          const translatedText = translate(originalText);
          if (translatedText !== currentText) { // Only update if different
            element.textContent = translatedText;
          }
        } else if (language === 'en' && originalText) {
          // Reset to original English content
          if (currentText !== originalText) { // Only update if different
            element.textContent = originalText;
          }
        }
      });
    };

    // Run translation when language changes and on initial render
    translateDocument();

    // Set up a MutationObserver to handle dynamically added content
    const observer = new MutationObserver(() => {
      // Process mutations after a brief delay to ensure content is rendered
      setTimeout(translateDocument, 100);
    });

    observer.observe(document.body, {
      childList: true,
      subtree: true
    });

    // Also set up a timer to periodically check for elements (in case of timing issues)
    const intervalId = setInterval(translateDocument, 2000); // Check every 2 seconds

    // Clean up the observer and interval when component unmounts
    return () => {
      observer.disconnect();
      clearInterval(intervalId);
    };
  }, [language, translate, location.pathname]); // Run whenever language, translate function, or route changes

  return null; // This component doesn't render anything itself
};

export default DOMTranslateHandler;
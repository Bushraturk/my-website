import React, { useEffect } from 'react';
import { useLanguage } from '../contexts/LanguageContext';

interface TranslateContentProps {
  children: React.ReactNode;
}

const TranslateContent: React.FC<TranslateContentProps> = ({ children }) => {
  const { language, translate } = useLanguage();

  useEffect(() => {
    // This effect would handle dynamic translation of content
    // However, for static MDX content, we need a different approach
    
    // For now, we'll just ensure the page re-renders when language changes
    // by using the language state in this component
  }, [language]);

  // Process the children to translate text content
  const processChildren = (node: React.ReactNode): React.ReactNode => {
    if (typeof node === 'string') {
      // Translate string content
      return translate(node);
    } else if (React.isValidElement(node)) {
      // Recursively process element children
      const element = node as React.ReactElement;
      const props: any = {};
      
      // Process children of this element
      if (element.props.children) {
        props.children = React.Children.map(element.props.children, processChildren);
      }
      
      // Copy other props
      Object.keys(element.props).forEach(key => {
        if (key !== 'children') {
          props[key] = (element.props as any)[key];
        }
      });

      return React.cloneElement(element, props);
    } else if (Array.isArray(node)) {
      // Process array of elements
      return node.map((child, index) => (
        <React.Fragment key={index}>{processChildren(child)}</React.Fragment>
      ));
    }
    
    return node;
  };

  return <>{processChildren(children)}</>;
};

export default TranslateContent;
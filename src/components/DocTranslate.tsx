import React from 'react';

interface DocTranslateProps {
  children: React.ReactNode;  // Changed to ReactNode to handle more complex children
  id?: string;
}

/**
 * Component to translate content within Docusaurus markdown files
 * This component renders the content with necessary attributes for DOM translation
 */
const DocTranslate: React.FC<DocTranslateProps> = ({ children, id }) => {
  // Convert ReactNode to text for the data-original attribute
  const getTextFromChildren = (children: React.ReactNode): string => {
    if (typeof children === 'string') {
      return children;
    } else if (typeof children === 'number') {
      return children.toString();
    } else if (React.isValidElement(children)) {
      // If it's a React element, try to extract its text content
      if (typeof children.props.children === 'string') {
        return children.props.children;
      } else if (typeof children.props.children === 'number') {
        return children.props.children.toString();
      }
      // For complex elements, we might need to extract text recursively
      return getTextFromChildren(children.props.children);
    } else if (Array.isArray(children)) {
      return children.map(getTextFromChildren).join(' ');
    }
    return '';
  };

  const originalText = getTextFromChildren(children);

  // Create a unique key but render with data attributes for DOM translator
  const key = `${id || 'doc-translate'}`;

  // Render a span with data attributes and CSS class for translation
  // This approach marks elements specifically for translation
  return (
    <span
      key={key}
      className="docusaurus-translate"
      data-docusaurus-translate="true"
      data-original={originalText}
    >
      {children}
    </span>
  );
};

export default DocTranslate;
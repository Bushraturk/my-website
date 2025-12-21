import React from 'react';

// Root component without auth providers to avoid recursive rendering during build
const Root = ({ children }) => {
  // Simply render children without any authentication wrappers during build
  // Authentication is handled in BrowserOnly components/pages
  return <>{children}</>;
};

export default Root;
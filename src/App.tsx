import React from 'react';

// App component without extra AuthProvider wrapper to avoid conflicts
const App = ({ children }) => {
  return children;
};

export default App;
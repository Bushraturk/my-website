import React from 'react';
import { useNavigate } from '@docusaurus/router';
import { client } from '../auth/client';

interface LogoutButtonProps {
  onLogout?: () => void; // Optional callback after logout
}

const LogoutButton: React.FC<LogoutButtonProps> = ({ onLogout }) => {
  const navigate = useNavigate();

  const handleLogout = async () => {
    try {
      // For static site implementation, we'll simulate the logout process
      // In a real implementation with server capabilities, this would call:
      // await client.signOut();
      
      // For now, we'll clear the mock user data from localStorage
      localStorage.removeItem('mockUser');
      localStorage.removeItem('activeSession');
      
      // Call the optional callback
      if (onLogout) {
        onLogout();
      }
      
      // Redirect to home page after logout
      navigate('/');
    } catch (error) {
      console.error('Error during logout:', error);
      // Even if there's an error, we should clear local data
      localStorage.removeItem('mockUser');
      localStorage.removeItem('activeSession');
      if (onLogout) {
        onLogout();
      }
      navigate('/');
    }
  };

  return (
    <button 
      onClick={handleLogout} 
      className="button button--secondary"
    >
      Logout
    </button>
  );
};

export default LogoutButton;

// Add some basic styling via CSS classes

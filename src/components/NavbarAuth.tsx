import React, { useState, useEffect } from 'react';
import Link from '@docusaurus/Link';
import BrowserOnly from '@docusaurus/BrowserOnly';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

// Inner component that safely uses browser APIs
const NavbarAuthContent: React.FC = () => {
  const [user, setUser] = useState<any>(null);
  const [loading, setLoading] = useState(true);
  const { siteConfig } = useDocusaurusContext();

  useEffect(() => {
    // Load user from localStorage
    const loadUser = () => {
      try {
        const mockUser = localStorage.getItem('mockUser');
        if (mockUser) {
          const userData = JSON.parse(mockUser);
          // Check if session is still valid
          const sessionExpires = new Date(userData.session?.expiresAt);
          if (new Date() < sessionExpires) {
            setUser(userData);
          } else {
            localStorage.removeItem('mockUser');
          }
        }
      } catch (error) {
        console.error('Error loading user:', error);
      } finally {
        setLoading(false);
      }
    };

    loadUser();
  }, []);

  const handleSignOut = () => {
    try {
      localStorage.removeItem('mockUser');
      localStorage.removeItem('activeSession');
      setUser(null);
      // Use baseUrl for correct redirect
      window.location.href = siteConfig.baseUrl;
    } catch (error) {
      console.error('Error signing out:', error);
    }
  };

  if (loading) {
    return null;
  }

  return (
    <div className="navbar-auth-container">
      {user ? (
        <div className="navbar-auth-logged-in">
          <Link to="/user-profile" className="navbar-auth-profile">
            <span className="navbar-auth-avatar">
              {(user.name || user.email || 'U').charAt(0).toUpperCase()}
            </span>
            <span className="navbar-auth-name">{user.name || 'Profile'}</span>
          </Link>
          <button onClick={handleSignOut} className="navbar-auth-logout">
            Logout
          </button>
        </div>
      ) : (
        <div className="navbar-auth-buttons">
          <Link to="/signup" className="navbar-auth-btn navbar-auth-btn-primary">
            Sign Up
          </Link>
          <Link to="/signin" className="navbar-auth-btn navbar-auth-btn-secondary">
            Sign In
          </Link>
        </div>
      )}
    </div>
  );
};

// Main component that wraps with BrowserOnly
const NavbarAuth: React.FC = () => {
  return (
    <BrowserOnly fallback={null}>
      {() => <NavbarAuthContent />}
    </BrowserOnly>
  );
};

export default NavbarAuth;

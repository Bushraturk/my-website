import React from 'react';
import { useAuth } from '../contexts/AuthContext';
import { Link } from '@docusaurus/router';

const LoginStatus: React.FC = () => {
  const { user, loading } = useAuth();

  if (loading) {
    return <div>Loading...</div>;
  }

  return (
    <div className="login-status">
      {user ? (
        <div className="user-info">
          <span>Welcome, {user.name || user.email}!</span>
          <Link to="/user-profile" className="profile-link">
            Profile
          </Link>
        </div>
      ) : (
        <div className="auth-options">
          <Link to="/signin" className="signin-link">
            Sign In
          </Link>
          <span> | </span>
          <Link to="/signup" className="signup-link">
            Sign Up
          </Link>
        </div>
      )}
    </div>
  );
};

export default LoginStatus;

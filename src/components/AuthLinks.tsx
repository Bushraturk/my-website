import React from 'react';
import { useAuth } from '../contexts/AuthContext';
import { useHistory, Link } from '@docusaurus/router';

const AuthLinks: React.FC = () => {
  const { user, loading, logout } = useAuth();
  const history = useHistory();

  const handleLogout = async () => {
    await logout();
    history.push('/'); // Redirect to home after logout
  };

  if (loading) {
    return (
      <div className="navbar__item">
        <span className="navbar__link">Loading...</span>
      </div>
    );
  }

  return (
    <div className="navbar__item">
      {user ? (
        <div className="dropdown dropdown--right dropdown--navbar">
          <span className="navbar__link dropdown__link">{user.name || user.email} ▾</span>
          <ul className="dropdown__menu">
            <li>
              <Link to="/user-profile" className="dropdown__link">Profile</Link>
            </li>
            <li>
              <button
                onClick={handleLogout}
                className="dropdown__link"
                style={{ width: '100%', textAlign: 'left', cursor: 'pointer', border: 'none', background: 'none' }}
              >
                Logout
              </button>
            </li>
          </ul>
        </div>
      ) : (
        <div className="navbar__items navbar__items--right">
          <Link to="/signin" className="button button--secondary navbar__link">Sign In</Link>
          <Link to="/signup" className="button button--primary navbar__link" style={{ marginLeft: '0.5rem' }}>Sign Up</Link>
        </div>
      )}
    </div>
  );
};

export default AuthLinks;
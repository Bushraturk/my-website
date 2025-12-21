import React from 'react';
import { useAuth } from '../contexts/AuthContext';
import { Link } from '@docusaurus/router';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

interface ProtectedContentProps {
  children: React.ReactNode;
  fallback?: React.ReactNode; // Content to show if not authenticated
  requireAuth?: boolean; // Whether authentication is required
  redirectPath?: string; // Where to redirect if not authenticated
}

const ProtectedContent: React.FC<ProtectedContentProps> = ({
  children,
  fallback = (
    <div>
      <p>You need to be logged in to view this content.</p>
      <Link to="/signin" className="button button--primary">
        Sign In
      </Link>
    </div>
  ),
  requireAuth = true,
  redirectPath = '/signin'
}) => {
  // Only use auth context in browser environment to avoid SSR issues
  if (!ExecutionEnvironment.canUseDOM) {
    // During SSR, just render children without protection
    return <>{children}</>;
  }

  // In browser environment, use the auth context
  const { isAuthenticated, loading } = useAuth();

  if (loading) {
    return <div>Loading...</div>;
  }

  if (requireAuth && !isAuthenticated) {
    return <>{fallback}</>;
  }

  if (!requireAuth && isAuthenticated) {
    // If auth is not required but user is logged in, may want different behavior
    // For now, return the fallback or nothing depending on use case
    return <>{children}</>;
  }

  return <>{children}</>;
};

export default ProtectedContent;

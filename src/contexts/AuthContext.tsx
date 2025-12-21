import React, { createContext, useContext, useEffect, useState, ReactNode } from 'react';

// Define the user type
interface User {
  id: string;
  email: string;
  name: string;
}

// Define the session type
interface Session {
  token: string;
  expiresAt: Date;
}

// Define the background information type
interface UserBackground {
  programmingExperience: 'beginner' | 'intermediate' | 'advanced' | null;
  hardwareExperience: 'none' | 'basic' | 'intermediate' | 'advanced' | null;
  roboticsBackground: 'none' | 'academic' | 'industry' | null;
  interests: string[]; // Array of interests like ['ROS 2', 'Simulation', 'NVIDIA Isaac', 'RL', 'Humanoids']
}

// Define the user data type
interface UserData {
  id: string;
  email: string;
  name: string;
  password?: string; // Only for mock implementation
  session: Session;
  background: UserBackground;
  // Include the individual background fields for compatibility with existing data structure
  programmingExperience?: 'beginner' | 'intermediate' | 'advanced' | null;
  hardwareExperience?: 'none' | 'basic' | 'intermediate' | 'advanced' | null;
  roboticsBackground?: 'none' | 'academic' | 'industry' | null;
  interests?: string[] | string; // Can be array or comma-separated string
}

// Define the context type
interface AuthContextType {
  user: UserData | null;
  loading: boolean;
  isAuthenticated: boolean;
  login: (userData: UserData) => void;
  logout: () => Promise<void>;
  updateBackground: (background: Partial<UserBackground>) => void;
}

// Default context value for SSR - prevents crashes when context is undefined
const defaultAuthContext: AuthContextType = {
  user: null,
  loading: false,
  isAuthenticated: false,
  login: () => {},
  logout: async () => {},
  updateBackground: () => {},
};

// Create the context with default value
const AuthContext = createContext<AuthContextType>(defaultAuthContext);

// Define the provider props
interface AuthProviderProps {
  children: ReactNode;
}

// Create the provider component
export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  // Prevent initializing state during SSR to avoid hydration issues
  const [user, setUser] = useState<UserData | null>(null);
  const [loading, setLoading] = useState<boolean>(typeof window !== 'undefined');

  // Load user data from localStorage on component mount (client-side only)
  useEffect(() => {
    if (typeof window !== 'undefined') {
      const loadUserData = () => {
        try {
          const mockUserData = localStorage.getItem('mockUser');

          if (mockUserData) {
            const parsedData = JSON.parse(mockUserData);

            // Check if session is still valid
            const sessionExpires = new Date(parsedData.session?.expiresAt);
            const now = new Date();

            if (now < sessionExpires) {
              // Update the user state with the loaded data, ensuring proper background structure
              const userData: UserData = {
                ...parsedData,
                session: {
                  ...parsedData.session,
                  expiresAt: new Date(parsedData.session.expiresAt) // Convert to Date object
                },
                background: {
                  programmingExperience: parsedData.background?.programmingExperience || parsedData.programmingExperience || null,
                  hardwareExperience: parsedData.background?.hardwareExperience || parsedData.hardwareExperience || null,
                  roboticsBackground: parsedData.background?.roboticsBackground || parsedData.roboticsBackground || null,
                  interests: Array.isArray(parsedData.background?.interests)
                    ? parsedData.background.interests
                    : (parsedData.background?.interests || parsedData.interests
                      ? String(parsedData.background?.interests || parsedData.interests).split(',').map((i: string) => i.trim())
                      : [])
                }
              };

              setUser(userData);
            } else {
              // Session expired, remove from localStorage
              localStorage.removeItem('mockUser');
            }
          }
        } catch (error) {
          console.error('Error loading user data:', error);
        } finally {
          setLoading(false);
        }
      };

      // Delay loading to ensure hydration is complete
      const timer = setTimeout(loadUserData, 0);
      return () => clearTimeout(timer);
    } else {
      setLoading(false);
    }
  }, []);

  // Login function
  const login = (userData: UserData) => {
    if (typeof window !== 'undefined') {
      setUser(userData);
      localStorage.setItem('mockUser', JSON.stringify(userData));
    }
  };

  // Logout function - using localStorage only approach for static site
  const logout = async () => {
    try {
      if (typeof window !== 'undefined') {
        localStorage.removeItem('mockUser');
        localStorage.removeItem('activeSession');
      }
    } catch (error) {
      console.error('Error during logout:', error);
    } finally {
      setUser(null);
    }
  };

  // Update background information
  const updateBackground = (backgroundUpdates: Partial<UserBackground>) => {
    if (user && typeof window !== 'undefined') {
      const updatedUser = {
        ...user,
        ...backgroundUpdates,
        background: {
          ...user.background,
          ...backgroundUpdates
        }
      };

      setUser(updatedUser);
      localStorage.setItem('mockUser', JSON.stringify(updatedUser));
    }
  };

  // Check if user is authenticated
  const isAuthenticated = !!user;

  return (
    <AuthContext.Provider
      value={{
        user,
        loading,
        isAuthenticated,
        login,
        logout,
        updateBackground,
      }}
    >
      {children}
    </AuthContext.Provider>
  );
};

// Custom hook to use the auth context - now safe for SSR
export const useAuth = (): AuthContextType => {
  const context = useContext(AuthContext);
  // No longer throws - returns default context if not wrapped in provider
  return context;
};

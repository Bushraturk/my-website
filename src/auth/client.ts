import { createAuthClient } from "better-auth/react";

// For static site implementation, we'll implement a custom client that works with simulated auth
// since we can't have actual API routes in a static site
// We'll implement a mock client that doesn't make actual network requests
export const client = {
  // Mock client implementation to avoid network calls during static generation
  getSession: () => {
    if (typeof window !== 'undefined') {
      const mockUser = localStorage.getItem('mockUser');
      return Promise.resolve(mockUser ? JSON.parse(mockUser) : null);
    }
    return Promise.resolve(null);
  },
  signIn: {
    email: (data: { email: string; password: string }) => {
      // Use mock sign in function
      return mockSignIn(data.email, data.password);
    }
  },
  signUp: {
    email: (data: { email: string; password: string; }) => {
      // Use mock sign up function that includes additional user data
      return mockSignUp(data.email, data.password, data);
    }
  },
  signOut: () => {
    // Use mock sign out function
    return mockSignOut();
  }
};

// Define types for user background information
export type UserBackground = {
  programmingExperience: 'beginner' | 'intermediate' | 'advanced' | null;
  hardwareExperience: 'none' | 'basic' | 'intermediate' | 'advanced' | null;
  roboticsBackground: 'none' | 'academic' | 'industry' | null;
  interests: string[]; // Array of interests like ['ROS 2', 'Simulation', 'NVIDIA Isaac', 'RL', 'Humanoids']
};

// For static site implementation, we'll implement mock authentication functions
// that use localStorage to simulate server-side auth functionality
export const mockSignIn = async (email: string, password: string) => {
  // Check if user exists in localStorage
  const mockUsers = JSON.parse(localStorage.getItem('mockUsers') || '{}');
  const user: any = mockUsers[email];

  if (user && user.password === password) {
    // Update the session to simulate login
    const userDataWithSession = {
      ...user,
      background: {
        programmingExperience: user.programmingExperience || null,
        hardwareExperience: user.hardwareExperience || null,
        roboticsBackground: user.roboticsBackground || null,
        interests: Array.isArray(user.interests) ? user.interests : (user.interests ? user.interests.split(',') : [])
      },
      session: {
        token: `session_${Date.now()}`,
        expiresAt: new Date(Date.now() + 7 * 24 * 60 * 60 * 1000).toISOString(), // 7 days
      }
    };

    localStorage.setItem('mockUser', JSON.stringify(userDataWithSession));
    return { user: userDataWithSession, error: null };
  } else {
    return { user: null, error: { message: 'Invalid email or password' } };
  }
};

export const mockSignUp = async (email: string, password: string, additionalData: any) => {
  // Check if user already exists
  const mockUsers = JSON.parse(localStorage.getItem('mockUsers') || '{}');

  if (mockUsers[email]) {
    return { user: null, error: { message: 'User already exists' } };
  }

  // Create new user
  const newUser = {
    id: `user_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`,
    email,
    name: email.split('@')[0], // Use part of email as name
    password, // Store password for validation (in a real app, this would be hashed server-side)
    programmingExperience: additionalData.programmingExperience || null,
    hardwareExperience: additionalData.hardwareExperience || null,
    roboticsBackground: additionalData.roboticsBackground || null,
    interests: additionalData.interests || [],
    createdAt: new Date().toISOString(),
    updatedAt: new Date().toISOString(),
  };

  // Store all users in localStorage
  mockUsers[email] = newUser;
  localStorage.setItem('mockUsers', JSON.stringify(mockUsers));

  // Also store current user session with background info
  const userDataWithSession = {
    ...newUser,
    background: {
      programmingExperience: newUser.programmingExperience,
      hardwareExperience: newUser.hardwareExperience,
      roboticsBackground: newUser.roboticsBackground,
      interests: newUser.interests
    },
    session: {
      token: `session_${Date.now()}`,
      expiresAt: new Date(Date.now() + 7 * 24 * 60 * 60 * 1000).toISOString(), // 7 days
    }
  };

  localStorage.setItem('mockUser', JSON.stringify(userDataWithSession));

  return { user: userDataWithSession, error: null };
};

export const mockSignOut = async () => {
  localStorage.removeItem('mockUser');
  localStorage.removeItem('activeSession');
};
// Export signOut as alias for mockSignOut
export const signOut = mockSignOut;

// Mock useSession hook for static site
// Returns a simple object instead of SWR-like interface
export const useSession = () => {
  // This should only be called in browser
  if (typeof window === 'undefined') {
    return { data: null, mutate: async () => {} };
  }
  
  const mockUser = localStorage.getItem('mockUser');
  const user = mockUser ? JSON.parse(mockUser) : null;
  
  return {
    data: user ? { user } : null,
    mutate: async () => {
      // Force re-read from localStorage
      window.location.reload();
    }
  };
};

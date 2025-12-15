/**
 * Custom hook for authentication state management
 */
import { useState, useEffect, useContext, createContext, ReactNode } from 'react';
import authService from '../services/auth';

// Define TypeScript interfaces
interface User {
  id: string;
  email: string;
  name: string;
  createdAt: string;
  educationLevel?: string;
  programmingExperience?: string;
  softwareBackground?: string;
  hardwareBackground?: string;
  roboticsBackground?: string;
}

interface Session {
  user: User;
  expiresAt: string;
}

interface AuthResponse {
  success: boolean;
  user?: User;
  session?: Session;
  message?: string;
}

interface AuthContextType {
  user: User | null;
  loading: boolean;
  error: string | null;
  signup: (userData: any) => Promise<AuthResponse>;
  signin: (credentials: { email: string; password: string }) => Promise<AuthResponse>;
  signout: () => Promise<AuthResponse>;
  getUserProfile: () => Promise<AuthResponse>;
  updateUserProfile: (profileData: any) => Promise<AuthResponse>;
  isAuthenticated: boolean;
}

// Create auth context with default value
const AuthContext = createContext<AuthContextType | undefined>(undefined);

// Auth provider component props
interface AuthProviderProps {
  children: ReactNode;
}

// Auth provider component
export const AuthProvider = ({ children }: AuthProviderProps) => {
  const [user, setUser] = useState<User | null>(null);
  const [loading, setLoading] = useState<boolean>(true);
  const [error, setError] = useState<string | null>(null);

  // Check session on initial load
  useEffect(() => {
    const checkSession = async () => {
      try {
        setLoading(true);
        setError(null);

        // First check localStorage for session data (stored after sign-in)
        const storedSession = localStorage.getItem('auth_session');
        if (storedSession) {
          try {
            const session = JSON.parse(storedSession);
            if (session && session.user) {
              setUser(session.user);
              setLoading(false);
              return;
            }
          } catch (e) {
            // Invalid JSON, continue to API check
          }
        }

        // Fallback: Check with auth server via API
        const sessionData = await authService.getSession();
        if (sessionData && sessionData.user) {
          setUser(sessionData.user);
        } else {
          setUser(null);
        }
      } catch (err: any) {
        console.error('Error checking session:', err);
        setError(err.message || 'Error checking session');
        setUser(null);
      } finally {
        setLoading(false);
      }
    };

    checkSession();

    // Set up interval to refresh session periodically (e.g., every 10 minutes)
    const interval = setInterval(async () => {
      if (user) { // Only refresh if user is logged in
        try {
          const sessionData = await authService.getSession();
          if (sessionData && sessionData.user) {
            setUser(sessionData.user);
          } else {
            // Session expired, log user out
            setUser(null);
          }
        } catch (err: any) {
          console.error('Error refreshing session:', err);
          setUser(null);
        }
      }
    }, 10 * 60 * 1000); // 10 minutes

    return () => clearInterval(interval);
  }, []);

  // Sign up function
  const signup = async (userData: any) => {
    try {
      setError(null);
      const response = await authService.signup(userData);
      if (response.user) {
        setUser(response.user);
      }
      return response;
    } catch (err: any) {
      const errorMessage = err.message || 'Signup failed';
      setError(errorMessage);
      throw err;
    }
  };

  // Sign in function
  const signin = async (credentials: { email: string; password: string }) => {
    try {
      setError(null);
      const response = await authService.signin(credentials);
      if (response.session?.user) {
        setUser(response.session.user);
      }
      return response;
    } catch (err: any) {
      const errorMessage = err.message || 'Signin failed';
      setError(errorMessage);
      throw err;
    }
  };

  // Sign out function
  const signout = async () => {
    try {
      setError(null);
      const response = await authService.signout();
      setUser(null);
      return response;
    } catch (err: any) {
      const errorMessage = err.message || 'Signout failed';
      setError(errorMessage);
      // Even if signout fails, clear local state
      setUser(null);
      throw err;
    }
  };

  // Get user profile
  const getUserProfile = async () => {
    try {
      setError(null);
      const response = await authService.getUserProfile();
      if (response && response.user) {
        setUser(response.user);
        return response;
      }
      return response;
    } catch (err: any) {
      const errorMessage = err.message || 'Failed to get user profile';
      setError(errorMessage);
      throw err;
    }
  };

  // Update user profile
  const updateUserProfile = async (profileData: any) => {
    try {
      setError(null);
      const response = await authService.updateUserProfile(profileData);
      if (response && response.user) {
        setUser(response.user);
        return response;
      }
      return response;
    } catch (err: any) {
      const errorMessage = err.message || 'Failed to update user profile';
      setError(errorMessage);
      throw err;
    }
  };

  // Value to provide through context
  const value: AuthContextType = {
    user,
    loading,
    error,
    signup,
    signin,
    signout,
    getUserProfile,
    updateUserProfile,
    isAuthenticated: !!user
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
};

// Custom hook to use auth context
export const useAuth = (): AuthContextType => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};

export default useAuth;
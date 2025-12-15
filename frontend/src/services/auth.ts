/**
 * Auth API client for Better Auth integration
 */
import axios, { AxiosInstance, AxiosResponse } from 'axios';

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

interface ErrorResponse {
  message: string;
  status?: number;
  details?: any;
}

interface ProfileUpdateData {
  name?: string;
  educationLevel?: string;
  programmingExperience?: string;
  softwareBackground?: string;
  hardwareBackground?: string;
  roboticsBackground?: string;
}

// Create axios instance with base configuration
// Create axios instance with base configuration
// Note: We avoid accessing process.env directly here to prevent ReferenceError in browser
const authClient: AxiosInstance = axios.create({
  baseURL: 'http://localhost:3001/api', // Default fallback
  withCredentials: true, // Important for cookie-based auth
  headers: {
    'Content-Type': 'application/json',
  },
});

// Add request interceptor to include auth token if available
authClient.interceptors.request.use(
  (config) => {
    // You can add additional headers or token handling here if needed
    return config;
  },
  (error) => {
    return Promise.reject(error);
  }
);

// Add response interceptor for error handling
authClient.interceptors.response.use(
  (response) => {
    return response;
  },
  (error) => {
    console.error('Auth API error:', error);
    return Promise.reject(error);
  }
);

// Auth API service
const authService = {
  // Configure the service with dynamic values from environment
  // This must be called from a component that has access to Docusaurus context
  setBaseUrl(url: string) {
    if (url) {
      authClient.defaults.baseURL = `${url}/api`;
    }
  },

  // Signup a new user
  async signup(userData: any): Promise<AuthResponse> {
    try {
      const response = await authClient.post<AuthResponse>('/auth/signup', userData);
      return response.data;
    } catch (error: any) {
      throw this.handleError(error);
    }
  },

  // Sign in an existing user
  async signin(credentials: { email: string; password: string }): Promise<AuthResponse> {
    try {
      const response = await authClient.post<AuthResponse>('/auth/signin', credentials);
      return response.data;
    } catch (error: any) {
      throw this.handleError(error);
    }
  },

  // Sign out the current user
  async signout(): Promise<AuthResponse> {
    try {
      const response = await authClient.post<AuthResponse>('/auth/signout');
      return response.data;
    } catch (error: any) {
      throw this.handleError(error);
    }
  },

  // Get current session
  async getSession(): Promise<Session | null> {
    try {
      const response = await authClient.get<AuthResponse>('/auth/get-session');
      return response.data.session || null;
    } catch (error: any) {
      // Return null if no session exists (401 error)
      if (error.response && error.response.status === 401) {
        return null;
      }
      throw this.handleError(error);
    }
  },

  // Get current user profile
  async getUserProfile(): Promise<AuthResponse> {
    try {
      const response = await authClient.get<AuthResponse>('/user/me');
      return response.data;
    } catch (error: any) {
      throw this.handleError(error);
    }
  },

  // Update user profile
  async updateUserProfile(profileData: ProfileUpdateData): Promise<AuthResponse> {
    try {
      const response = await authClient.put<AuthResponse>('/user/me', profileData);
      return response.data;
    } catch (error: any) {
      throw this.handleError(error);
    }
  },

  // Error handling helper
  handleError(error: any): ErrorResponse {
    if (error.response) {
      // Server responded with error status
      return {
        message: error.response.data?.error || 'An error occurred',
        status: error.response.status,
        details: error.response.data?.details
      };
    } else if (error.request) {
      // Request was made but no response received
      return {
        message: 'Network error - could not reach auth server',
        status: null
      };
    } else {
      // Something else happened
      return {
        message: error.message || 'An unexpected error occurred',
        status: null
      };
    }
  }
};

export default authService;
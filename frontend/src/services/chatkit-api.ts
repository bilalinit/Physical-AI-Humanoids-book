import axios, { AxiosInstance } from 'axios';

interface ChatThread {
  id: string;
  createdAt: string;
  updatedAt: string;
  title?: string;
  messageCount: number;
}

interface ChatMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: string;
  sources?: Array<{
    title: string;
    url: string;
    content?: string;
  }>;
}

interface ChatThreadDetail {
  id: string;
  messages: ChatMessage[];
}

interface UserProfile {
  education: string;
  experience: string;
}

interface ChatRequest {
  content: string;
  userProfile?: UserProfile;
}

interface ChatResponse {
  message: ChatMessage;
}

interface RAGQueryRequest {
  query: string;
  userProfile?: UserProfile;
}

interface RAGQueryResponse {
  response: string;
  sources: Array<{
    title: string;
    url: string;
    relevance: number;
  }>;
  processingTime: number;
}

interface HealthResponse {
  status: string;
  timestamp: string;
  services: {
    qdrant: string;
    openai_agents: string;
    authentication: string;
  };
}

class ChatKitApiService {
  private apiClient: AxiosInstance;

  constructor(baseURL: string = 'http://localhost:8000') {
    this.apiClient = axios.create({
      baseURL,
      timeout: 30000, // 30 second timeout
      headers: {
        'Content-Type': 'application/json',
      },
    });

    // Add request interceptor to include auth token
    this.apiClient.interceptors.request.use(
      (config) => {
        const token = this.getAuthToken();
        if (token) {
          config.headers.Authorization = `Bearer ${token}`;
        }
        return config;
      },
      (error) => {
        return Promise.reject(error);
      }
    );

    // Add response interceptor to handle errors globally
    this.apiClient.interceptors.response.use(
      (response) => response,
      (error) => {
        console.error('API Error:', error);
        // Handle specific error cases
        if (error.response?.status === 401) {
          // Handle unauthorized access
          this.handleAuthError();
        }
        return Promise.reject(error);
      }
    );
  }

  // Authentication helper methods
  private getAuthToken(): string | null {
    // Try multiple possible locations for the auth token
    return (
      localStorage.getItem('authToken') ||
      sessionStorage.getItem('authToken') ||
      // If using Better Auth, might be stored differently
      localStorage.getItem('better-auth.session_token') ||
      null
    );
  }

  private handleAuthError(): void {
    // Clear invalid auth token
    localStorage.removeItem('authToken');
    sessionStorage.removeItem('authToken');
    localStorage.removeItem('better-auth.session_token');

    // Optionally redirect to login page or show notification
    console.warn('Authentication required. Please log in.');
  }

  // Method to set auth token externally
  setAuthToken(token: string): void {
    localStorage.setItem('authToken', token);
  }

  // Method to clear auth token
  clearAuthToken(): void {
    localStorage.removeItem('authToken');
    sessionStorage.removeItem('authToken');
    localStorage.removeItem('better-auth.session_token');
  }

  // Get list of user's chat threads
  async getThreads(): Promise<{ threads: ChatThread[] }> {
    try {
      const response = await this.apiClient.get('/api/chat/threads');
      return response.data;
    } catch (error) {
      console.error('Error fetching threads:', error);
      throw error;
    }
  }

  // Get messages for a specific thread
  async getThreadMessages(threadId: string): Promise<{ thread: ChatThreadDetail }> {
    try {
      const response = await this.apiClient.get(`/api/chat/threads/${threadId}`);
      return response.data;
    } catch (error) {
      console.error(`Error fetching messages for thread ${threadId}:`, error);
      throw error;
    }
  }

  // Create a new chat thread
  async createThread(initialMessage?: string): Promise<{ threadId: string; createdAt: string }> {
    try {
      const response = await this.apiClient.post('/api/chat/threads', {
        initialMessage: initialMessage || 'New conversation'
      });
      return response.data;
    } catch (error) {
      console.error('Error creating thread:', error);
      throw error;
    }
  }

  // Send a message and receive AI response
  async sendMessage(threadId: string, request: ChatRequest): Promise<ChatResponse> {
    try {
      const response = await this.apiClient.post(`/api/chat/threads/${threadId}/messages`, request);
      return response.data;
    } catch (error) {
      console.error(`Error sending message to thread ${threadId}:`, error);
      throw error;
    }
  }

  // Process a RAG query without creating a thread
  async processRAGQuery(request: RAGQueryRequest): Promise<RAGQueryResponse> {
    try {
      const response = await this.apiClient.post('/api/chat/query', request);
      return response.data;
    } catch (error) {
      console.error('Error processing RAG query:', error);
      throw error;
    }
  }

  // Health check endpoint
  async healthCheck(): Promise<HealthResponse> {
    try {
      const response = await this.apiClient.get('/health');
      return response.data;
    } catch (error) {
      console.error('Health check error:', error);
      throw error;
    }
  }
}

export default ChatKitApiService;
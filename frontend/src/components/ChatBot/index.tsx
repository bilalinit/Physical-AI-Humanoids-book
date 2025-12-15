import React, { useState, useEffect, useRef } from 'react';
import { ChatKit, useChatKit } from '@openai/chatkit-react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import authService from '@site/src/services/auth';
import LoginPrompt from './LoginPrompt';

interface ChatBotProps {
  initialSelectedText?: string;
  pendingMessage?: string | null;
  onMessageSent?: () => void;
}

type AuthState = 'loading' | 'authenticated' | 'unauthenticated';

// Define the ChatBot component with ChatKit integration
const ChatBot: React.FC<ChatBotProps> = ({ initialSelectedText, pendingMessage, onMessageSent }) => {
  // Get API URL from Docusaurus config
  const { siteConfig } = useDocusaurusContext();
  const apiUrl = (siteConfig.customFields?.apiUrl as string) || 'http://localhost:8000';

  // Extract domain for ChatKit domainKey
  const getDomainKey = (url: string): string => {
    try {
      const hostname = new URL(url).hostname;
      return hostname;
    } catch {
      return 'localhost';
    }
  };

  // Determine color mode by checking the document class to avoid context errors
  const [colorMode, setColorMode] = useState<'light' | 'dark'>('light');

  useEffect(() => {
    // Check for dark mode class on document element
    const isDarkMode = document.documentElement.classList.contains('dark') ||
      (document.documentElement.getAttribute('data-theme') === 'dark');
    setColorMode(isDarkMode ? 'dark' : 'light');

    // Watch for theme changes
    const observer = new MutationObserver((mutations) => {
      mutations.forEach((mutation) => {
        if (mutation.type === 'attributes' && mutation.attributeName === 'class') {
          const isDark = document.documentElement.classList.contains('dark') ||
            (document.documentElement.getAttribute('data-theme') === 'dark');
          setColorMode(isDark ? 'dark' : 'light');
        }
      });
    });

    observer.observe(document.documentElement, {
      attributes: true,
      attributeFilter: ['class', 'data-theme']
    });

    return () => observer.disconnect();
  }, []);

  const [initialThread, setInitialThread] = useState<string | null>(null);
  const [isReady, setIsReady] = useState(false);

  // Authentication state
  const [authState, setAuthState] = useState<AuthState>('loading');

  // Check authentication on component mount
  useEffect(() => {
    const checkAuth = async () => {
      try {
        // First check localStorage for session data (stored after sign-in)
        const storedSession = localStorage.getItem('auth_session');
        if (storedSession) {
          try {
            const session = JSON.parse(storedSession);
            if (session && session.user) {
              setAuthState('authenticated');
              return;
            }
          } catch (e) {
            // Invalid JSON, continue to API check
          }
        }

        // Fallback: Check with auth server via API
        const session = await authService.getSession();
        if (session) {
          setAuthState('authenticated');
        } else {
          setAuthState('unauthenticated');
        }
      } catch (error) {
        console.error('Auth check failed:', error);
        setAuthState('unauthenticated');
      }
    };

    checkAuth();
  }, []);

  // Initialize ChatKit only after successful auth
  useEffect(() => {
    if (authState === 'authenticated') {
      const savedThread = localStorage.getItem('chatkit-thread-id');
      setInitialThread(savedThread);
      setIsReady(true);
    }
  }, [authState]);

  // State for ChatKit error handling
  const [chatKitError, setChatKitError] = useState<string | null>(null);

  // Configure ChatKit with error handling
  const { control, setComposerValue, focusComposer } = useChatKit({
    api: {
      url: apiUrl,
      domainKey: 'localhost', // Use localhost for custom backend to bypass OpenAI domain verification
    },
    initialThread: initialThread,
    theme: {
      colorScheme: colorMode, // Use Docusaurus color mode
      color: {
        grayscale: { hue: 220, tint: 6, shade: -1 },
        accent: { primary: '#4cc9f0', level: 1 },
      },
      radius: 'round',
    },
    startScreen: {
      greeting: 'Welcome! Ask me anything about the documentation.',
      prompts: [
        { label: 'Hello', prompt: 'Say hello and introduce yourself' },
        { label: 'Help', prompt: 'What can you help me with?' },
        { label: 'Documentation', prompt: 'Tell me about the project documentation' },
      ],
    },
    composer: {
      placeholder: 'Type a message about the documentation...',
    },
    onThreadChange: ({ threadId }) => {
      if (threadId) {
        localStorage.setItem('chatkit-thread-id', threadId);
      }
    },
    onError: ({ error }) => {
      console.error('ChatKit error:', error);
      setChatKitError(error.message || 'An error occurred with ChatKit');
    },
  });

  // Pre-fill composer when pendingMessage (selected text) is provided using ChatKit API
  useEffect(() => {
    if (pendingMessage && isReady && setComposerValue) {
      // Extract just the selected text without "Explain this:" prefix
      const selectedText = pendingMessage.replace('Explain this: ', '');
      // Format with context marker for backend
      const formattedMessage = `[CONTEXT: ${selectedText}] Explain this`;

      // Use ChatKit's proper API to set the composer value
      setComposerValue({ text: formattedMessage });
      if (focusComposer) {
        focusComposer();
      }
      onMessageSent?.();
    }
  }, [pendingMessage, isReady, setComposerValue, focusComposer, onMessageSent]);

  // Wait for authentication check
  if (authState === 'loading') {
    return (
      <div
        style={{
          width: '100%',
          height: '100%',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          backgroundColor: colorMode === 'dark' ? 'var(--ifm-background-surface-color)' : 'var(--ifm-color-white)',
        }}
      >
        <div>
          <div>Checking authentication...</div>
        </div>
      </div>
    );
  }

  // Show login prompt if not authenticated
  if (authState === 'unauthenticated') {
    return <LoginPrompt colorMode={colorMode} />;
  }

  // Wait for ChatKit initialization (only reached if authenticated)
  if (!isReady) {
    return (
      <div
        style={{
          width: '100%',
          height: '100%',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          backgroundColor: colorMode === 'dark' ? 'var(--ifm-background-surface-color)' : 'var(--ifm-color-white)',
        }}
      >
        <div>
          <div>Loading chat interface...</div>
        </div>
      </div>
    );
  }

  // Show error if ChatKit failed to initialize
  if (chatKitError) {
    return (
      <div
        style={{
          width: '100%',
          height: '100%',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          backgroundColor: colorMode === 'dark' ? 'var(--ifm-background-surface-color)' : 'var(--ifm-color-white)',
          color: 'red',
          padding: '20px',
          textAlign: 'center'
        }}
      >
        <div>
          <div>Error loading chat interface</div>
          <div style={{ fontSize: '14px', marginTop: '10px' }}>
            {chatKitError}
          </div>
          <button
            onClick={() => window.location.reload()}
            style={{
              marginTop: '15px',
              padding: '8px 16px',
              backgroundColor: '#4361ee',
              color: 'white',
              border: 'none',
              borderRadius: '4px',
              cursor: 'pointer'
            }}
          >
            Retry
          </button>
        </div>
      </div>
    );
  }

  // Return the ChatKit component - fills available space
  return (
    <div
      style={{
        width: '100%',
        height: '100%',
        display: 'flex',
        flexDirection: 'column',
        overflow: 'hidden',
        backgroundColor: colorMode === 'dark' ? 'var(--ifm-background-surface-color)' : 'var(--ifm-color-white)',
      }}
    >
      {/* Header with New Chat button */}
      <div
        style={{
          padding: '10px 16px',
          borderBottom: '1px solid var(--ifm-color-emphasis-300)',
          backgroundColor: colorMode === 'dark' ? 'var(--ifm-color-emphasis-100)' : 'var(--ifm-color-gray-100)',
          display: 'flex',
          justifyContent: 'flex-end',
          alignItems: 'center',
          flexShrink: 0,
        }}
      >
        <button
          onClick={() => {
            localStorage.removeItem('chatkit-thread-id');
            window.location.reload();
          }}
          style={{
            padding: '6px 12px',
            background: colorMode === 'dark' ? '#4361ee' : 'var(--ifm-color-emphasis-200)',
            color: colorMode === 'dark' ? 'white' : 'var(--ifm-color-emphasis-900)',
            border: '1px solid var(--ifm-color-emphasis-300)',
            borderRadius: '6px',
            cursor: 'pointer',
            fontSize: '12px',
            fontWeight: 500,
          }}
        >
          New Chat
        </button>
      </div>

      {/* ChatKit component - fills remaining space */}
      <div style={{ flex: 1, overflow: 'hidden', minHeight: 0 }}>
        <ChatKit
          control={control}
          style={{ height: '100%', width: '100%' }}
        />
      </div>
    </div>
  );
};

export default ChatBot;
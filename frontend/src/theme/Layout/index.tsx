import React, { useState, useEffect } from 'react';
import OriginalLayout from '@theme-original/Layout';
import ChatBot from '@site/src/components/ChatBot';
import type { Props } from '@theme/Layout';

// Floating ChatBot button component
const FloatingChatButton = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [pendingMessage, setPendingMessage] = useState<string | null>(null);

  // Listen for "Ask AI" events from text selection
  useEffect(() => {
    const handleAskAIEvent = (event: CustomEvent<{ message: string; text: string }>) => {
      if (event.detail && event.detail.message) {
        // Set the pending message and open the chat
        setPendingMessage(event.detail.message);
        setIsOpen(true);
      }
    };

    window.addEventListener('askAIWithSelection', handleAskAIEvent as EventListener);
    return () => window.removeEventListener('askAIWithSelection', handleAskAIEvent as EventListener);
  }, []);

  // Close chat when clicking outside
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      const chatElement = document.querySelector('[data-chatbot-container]');
      const buttonElement = document.querySelector('[data-chatbot-button]');

      if (isOpen && chatElement && buttonElement &&
        !chatElement.contains(event.target as Node) &&
        !buttonElement.contains(event.target as Node)) {
        setIsOpen(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, [isOpen]);

  // Clear pending message after it's been processed
  const handleMessageSent = () => {
    setPendingMessage(null);
  };

  return (
    <>

      {/* Floating ChatBot button */}
      <div
        data-chatbot-button
        style={{
          position: 'fixed',
          bottom: '20px',
          right: '20px',
          zIndex: 9999,
        }}
      >
        {!isOpen ? (
          <button
            onClick={() => setIsOpen(true)}
            style={{
              width: '60px',
              height: '60px',
              borderRadius: '50%',
              backgroundColor: '#4361ee',
              color: 'white',
              border: 'none',
              cursor: 'pointer',
              boxShadow: '0 4px 12px rgba(0,0,0,0.15)',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
              fontSize: '24px',
              fontWeight: 'bold',
            }}
          >
            💬
          </button>
        ) : (
          <div
            data-chatbot-container
            style={{
              width: '420px',
              height: '650px',
              maxHeight: 'calc(100vh - 120px)',
              position: 'absolute',
              bottom: '80px',
              right: '0',
              backgroundColor: 'var(--ifm-background-color)',
              borderRadius: '16px',
              boxShadow: '0 12px 48px rgba(0,0,0,0.25)',
              overflow: 'hidden',
              border: '1px solid var(--ifm-color-emphasis-300)',
              display: 'flex',
              flexDirection: 'column',
            }}
          >
            <div
              style={{
                padding: '14px 16px',
                backgroundColor: 'var(--ifm-color-emphasis-100)',
                borderBottom: '1px solid var(--ifm-color-emphasis-300)',
                display: 'flex',
                justifyContent: 'space-between',
                alignItems: 'center',
                flexShrink: 0,
              }}
            >
              <span style={{ fontWeight: 'bold' }}>Documentation Assistant</span>
              <button
                onClick={() => setIsOpen(false)}
                style={{
                  background: 'none',
                  border: 'none',
                  fontSize: '18px',
                  cursor: 'pointer',
                  color: '#666',
                }}
              >
                ×
              </button>
            </div>
            <div style={{ flex: 1, overflow: 'hidden', minHeight: 0 }}>
              <ChatBot
                pendingMessage={pendingMessage}
                onMessageSent={handleMessageSent}
              />
            </div>
          </div>
        )}
      </div>
    </>
  );
};

export default function Layout(props: Props) {
  return (
    <>
      <OriginalLayout {...props} />
      <FloatingChatButton />
    </>
  );
}
import React, { useState } from 'react';

interface RAGResponse {
  output: string;
  context_chunks: Array<{
    filename: string;
    text: string;
    chunk_number: number;
    score: number;
  }>;
  sources: string[];
}

const ChatBot = () => {
  const [query, setQuery] = useState('');
  const [response, setResponse] = useState<RAGResponse | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handleChatSubmit = async () => {
    setLoading(true);
    setError(null);
    try {
      const res = await fetch('http://localhost:8000/api/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          user_query: query,
          selected_text: null, // if any selected text
          chat_history: [] // if implementing chat history
        }),
      });

      if (!res.ok) {
        throw new Error(`API request failed with status ${res.status}`);
      }

      const data: RAGResponse = await res.json();
      setResponse(data);
    } catch (error) {
      console.error('Error:', error);
      setError(error instanceof Error ? error.message : 'An unknown error occurred');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="chatbot-container">
      <div className="chat-input">
        <input
          type="text"
          value={query}
          onChange={(e) => setQuery(e.target.value)}
          placeholder="Ask about the book content..."
          onKeyPress={(e) => e.key === 'Enter' && handleChatSubmit()}
        />
        <button onClick={handleChatSubmit} disabled={loading}>
          {loading ? 'Processing...' : 'Send'}
        </button>
      </div>

      {loading && <div className="loading">Processing your query...</div>}

      {error && <div className="error">Error: {error}</div>}

      {response && (
        <div className="chat-response">
          <div className="response-text">{response.output}</div>

          {response.sources.length > 0 && (
            <div className="sources-section">
              <h4>Sources:</h4>
              <ul>
                {response.sources.map((source, idx) => (
                  <li key={idx}>{source}</li>
                ))}
              </ul>
            </div>
          )}

          {response.context_chunks.length > 0 && (
            <div className="context-section">
              <h4>Context used:</h4>
              {response.context_chunks.map((chunk, idx) => (
                <div key={idx} className="context-chunk">
                  <p><strong>File:</strong> {chunk.filename}</p>
                  <p><strong>Relevance:</strong> {(chunk.score * 100).toFixed(1)}%</p>
                  <p><strong>Text:</strong> {chunk.text.substring(0, 100)}...</p>
                </div>
              ))}
            </div>
          )}
        </div>
      )}
    </div>
  );
};

export default ChatBot;
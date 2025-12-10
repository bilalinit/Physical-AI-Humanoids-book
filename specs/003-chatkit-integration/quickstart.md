# Quickstart Guide: ChatKit Integration

## Prerequisites

- Node.js 20+ installed
- Python 3.12+ with `uv` package manager
- Docusaurus 3.x project set up
- Existing FastAPI backend with RAG logic
- Qdrant vector database running
- Better Auth server running

## Setup Environment

1. **Clone the repository** (if not already done):
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Install frontend dependencies**:
   ```bash
   cd frontend
   npm install
   # Install ChatKit dependencies
   npm install @openai/chatkit
   ```

3. **Install backend dependencies**:
   ```bash
   cd backend
   uv venv
   source .venv/bin/activate  # On Windows: .venv\Scripts\activate
   uv pip install -r requirements.txt
   ```

4. **Install auth server dependencies**:
   ```bash
   cd auth-server
   npm install
   ```

5. **Set up environment variables**:
   Create `.env` files in each service directory based on `.env.example`:
   - `frontend/.env`
   - `backend/.env`
   - `auth-server/.env`

## Running the Application

1. **Start the Qdrant vector database**:
   ```bash
   docker-compose up -d  # if using Docker
   # Or start Qdrant via your preferred method
   ```

2. **Run the auth server** (port 3001):
   ```bash
   cd auth-server
   npm run dev
   ```

3. **Run the backend/FastAPI server** (port 8000):
   ```bash
   cd backend
   uv run uvicorn main:app --reload --port 8000
   ```

4. **Ingest documentation** (for RAG functionality):
   ```bash
   cd backend
   uv run python ingest.py
   ```

5. **Run the Docusaurus frontend** (port 3000):
   ```bash
   cd frontend
   npm run start
   ```

## Integration Steps

### 1. Frontend UI Replacement
Replace the existing custom chat components with ChatKit components:

1. Remove the existing `ChatBot/index.tsx` and `ChatBot.module.css`
2. Install ChatKit dependencies
3. Configure ChatKit provider in your Docusaurus app
4. Implement ChatKit interface components
5. Style components to match Docusaurus theme

### 2. Backend API Adaptation
Adapt existing FastAPI endpoints to work with ChatKit:

1. Review existing `main.py` endpoints
2. Adapt API responses to match ChatKit expectations
3. Ensure authentication continues to work
4. Maintain RAG logic and Qdrant integration
5. Preserve OpenAI Agent functionality

### 3. Thread Management
Implement ChatKit's store for thread management:

1. Configure ChatKit store for message persistence
2. Implement thread creation and retrieval
3. Ensure conversation history persists across sessions
4. Handle anonymous vs authenticated user threads

### 4. Agent Memory Integration
Ensure OpenAI Agents work with ChatKit:

1. Maintain context across ChatKit conversations
2. Preserve guardrails and safety checks
3. Ensure user profile data injection continues
4. Test multi-turn conversation memory

## Testing the Integration

1. **Start all services** as described above

2. **Open the Docusaurus site** at `http://localhost:3000`

3. **Test basic functionality**:
   - Chat interface loads properly
   - Messages can be sent and received
   - RAG responses work (ask questions about documentation)
   - Authentication works as expected

4. **Test thread persistence**:
   - Start a conversation
   - Refresh the page
   - Verify conversation history is preserved

5. **Test RAG functionality**:
   - Ask questions about documentation content
   - Verify responses include relevant information from vector search
   - Check response times are acceptable (<5 seconds)

## Troubleshooting

### Common Issues

**ChatKit components not loading**:
- Verify ChatKit dependencies are installed
- Check that API keys are properly configured
- Ensure CORS settings allow communication between services

**RAG responses not working**:
- Verify Qdrant database is running and populated
- Check that `ingest.py` has been run with current documentation
- Confirm FastAPI endpoints are properly adapted for ChatKit

**Authentication problems**:
- Verify JWT tokens are being passed correctly
- Check that auth server is running on port 3001
- Ensure backend validates tokens properly

### Useful Commands

**Check service status**:
```bash
# Check if Qdrant is running
curl http://localhost:6333/dashboard

# Test backend API
curl http://localhost:8000/health

# Check auth server
curl http://localhost:3001/api/auth/health
```

**Debug ChatKit integration**:
- Use browser dev tools to check network requests
- Verify ChatKit API calls are being made correctly
- Check console for any JavaScript errors
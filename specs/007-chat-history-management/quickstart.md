# Quickstart Guide: Chat History Database Persistence

## Overview
This guide provides a quick overview of how to implement and use the chat history persistence feature with PostgreSQL database storage.

## Prerequisites
- Python 3.12+ with `uv` package manager
- PostgreSQL database (Neon recommended)
- Running auth server for user authentication
- FastAPI backend server

## Setup Instructions

### 1. Database Setup
1. Set up your PostgreSQL database (Neon DB recommended)
2. Update your `.env` file with database connection details:
   ```
   DATABASE_URL=postgresql://username:password@host:port/database_name
   ```
3. Run database migrations:
   ```bash
   cd backend
   uv run alembic upgrade head
   ```

### 2. Backend Configuration
1. Install backend dependencies:
   ```bash
   cd backend
   uv sync
   ```
2. Ensure the following environment variables are set:
   - `DATABASE_URL`: PostgreSQL connection string
   - `AUTH_JWT_SECRET`: Secret for JWT validation
   - `API_URL`: Base URL for the API

## Key Components

### Database Models
- `ChatThread`: Represents a conversation session with user association
- `ChatMessage`: Represents individual messages within a thread
- Located in `backend/src/models/`

### Services
- `chat_history.py`: Core logic for chat history operations
- `database.py`: Database connection and session management
- Located in `backend/src/services/`

### API Endpoints
- `POST /api/v1/chat/threads`: Create new chat thread
- `GET /api/v1/chat/threads`: Get user's chat threads
- `GET /api/v1/chat/threads/{id}`: Get specific thread with messages
- `POST /api/v1/chat/threads/{id}/messages`: Add message to thread
- Located in `backend/src/api/chat.py`

## Usage Examples

### Creating a New Chat Thread
```python
import requests

# With valid JWT token
headers = {
    "Authorization": "Bearer <JWT_TOKEN>",
    "Content-Type": "application/json"
}

response = requests.post(
    "http://localhost:8000/api/v1/chat/threads",
    json={"title": "My New Conversation"},
    headers=headers
)

thread = response.json()
print(f"Created thread: {thread['id']}")
```

### Adding a Message to a Thread
```python
import requests

headers = {
    "Authorization": "Bearer <JWT_TOKEN>",
    "Content-Type": "application/json"
}

message_data = {
    "role": "user",
    "content": "Hello, how can I use this feature?",
    "sources": [{"type": "document", "id": "doc-123", "title": "Guide"}],
    "selected_text_used": False
}

response = requests.post(
    f"http://localhost:8000/api/v1/chat/threads/{thread_id}/messages",
    json=message_data,
    headers=headers
)

message = response.json()
print(f"Added message: {message['id']}")
```

### Retrieving Chat History
```python
import requests

headers = {
    "Authorization": "Bearer <JWT_TOKEN>"
}

response = requests.get(
    "http://localhost:8000/api/v1/chat/threads",
    headers=headers
)

threads = response.json()
for thread in threads['threads']:
    print(f"Thread: {thread['title']} - Updated: {thread['updated_at']}")
```

## Testing
Run the backend tests to verify the chat history functionality:
```bash
cd backend
uv run pytest tests/ -v
```

## Error Handling
- Database connection errors return 500 status codes
- Unauthorized access attempts return 401 status codes
- Resource not found returns 404 status codes
- Validation errors return 400 status codes

## Next Steps
1. Implement frontend integration to display chat history
2. Add pagination for threads with many messages
3. Implement thread search functionality
4. Add message editing/deletion capabilities
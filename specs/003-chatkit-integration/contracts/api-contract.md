# API Contract: ChatKit Integration

## Overview

This document defines the API contracts required for integrating OpenAI ChatKit with the existing RAG chatbot backend. The contract maintains compatibility with existing RAG functionality while adapting to ChatKit's expected API format.

## Authentication

All API endpoints require authentication via JWT token passed in the Authorization header:

```
Authorization: Bearer <jwt_token>
```

For unauthenticated users, endpoints should still function but with limited capabilities.

## API Endpoints

### Chat Thread Management

#### GET /api/chat/threads
**Description**: Retrieve list of user's chat threads

**Request**:
```
GET /api/chat/threads
Authorization: Bearer <jwt_token>
```

**Response**:
```json
{
  "threads": [
    {
      "id": "thread_abc123",
      "createdAt": "2025-12-09T10:00:00Z",
      "updatedAt": "2025-12-09T11:30:00Z",
      "title": "Documentation Questions",
      "messageCount": 5
    }
  ]
}
```

**Status Codes**:
- 200: Success
- 401: Unauthorized
- 500: Server error

#### GET /api/chat/threads/{thread_id}
**Description**: Retrieve messages for a specific thread

**Request**:
```
GET /api/chat/threads/{thread_id}
Authorization: Bearer <jwt_token>
```

**Response**:
```json
{
  "thread": {
    "id": "thread_abc123",
    "messages": [
      {
        "id": "msg_1",
        "role": "user",
        "content": "How do I set up the project?",
        "timestamp": "2025-12-09T10:00:00Z"
      },
      {
        "id": "msg_2",
        "role": "assistant",
        "content": "To set up the project, first install dependencies...",
        "timestamp": "2025-12-09T10:01:00Z",
        "sources": [
          {
            "title": "Getting Started Guide",
            "url": "/docs/getting-started",
            "content": "Detailed setup instructions..."
          }
        ]
      }
    ]
  }
}
```

**Status Codes**:
- 200: Success
- 401: Unauthorized
- 404: Thread not found
- 500: Server error

#### POST /api/chat/threads
**Description**: Create a new chat thread

**Request**:
```
POST /api/chat/threads
Authorization: Bearer <jwt_token>
Content-Type: application/json

{
  "initialMessage": "Hello"
}
```

**Response**:
```json
{
  "threadId": "thread_def456",
  "createdAt": "2025-12-09T12:00:00Z"
}
```

**Status Codes**:
- 201: Created
- 401: Unauthorized
- 500: Server error

### Chat Message Processing

#### POST /api/chat/threads/{thread_id}/messages
**Description**: Send a message and receive AI response with RAG-enhanced content

**Request**:
```
POST /api/chat/threads/{thread_id}/messages
Authorization: Bearer <jwt_token>
Content-Type: application/json

{
  "content": "What are the core principles of the project?",
  "userProfile": {
    "education": "beginner",
    "experience": "software:2_years"
  }
}
```

**Response**:
```json
{
  "message": {
    "id": "msg_xyz789",
    "role": "assistant",
    "content": "The core principles of the project include service isolation...",
    "timestamp": "2025-12-09T12:05:00Z",
    "sources": [
      {
        "title": "Project Constitution",
        "url": "/docs/constitution",
        "content": "Core principles: Service Isolation & Distinct Runtimes..."
      }
    ]
  }
}
```

**Status Codes**:
- 200: Success
- 401: Unauthorized
- 404: Thread not found
- 422: Content safety violation
- 500: Server error

### RAG Query Processing

#### POST /api/chat/query
**Description**: Process a RAG query without creating a thread (for testing purposes)

**Request**:
```
POST /api/chat/query
Authorization: Bearer <jwt_token>
Content-Type: application/json

{
  "query": "How do I configure authentication?",
  "userProfile": {
    "education": "intermediate",
    "experience": "software:5_years"
  }
}
```

**Response**:
```json
{
  "response": "Authentication is configured through the Better Auth server...",
  "sources": [
    {
      "title": "Authentication Guide",
      "url": "/docs/auth",
      "relevance": 0.92
    }
  ],
  "processingTime": 1.2
}
```

**Status Codes**:
- 200: Success
- 401: Unauthorized
- 422: Content safety violation
- 500: Server error

### Health and Status

#### GET /api/health
**Description**: Check the health status of the chat service

**Request**:
```
GET /api/health
```

**Response**:
```json
{
  "status": "healthy",
  "timestamp": "2025-12-09T12:00:00Z",
  "services": {
    "qdrant": "connected",
    "openai_agents": "available",
    "authentication": "available"
  }
}
```

**Status Codes**:
- 200: Healthy
- 503: Service unavailable

## Error Responses

All error responses follow this format:

```json
{
  "error": {
    "code": "ERROR_CODE",
    "message": "Human-readable error message",
    "details": "Additional error details if applicable"
  }
}
```

### Common Error Codes

- `AUTHENTICATION_REQUIRED`: User needs to authenticate
- `THREAD_NOT_FOUND`: Specified thread doesn't exist
- `CONTENT_VIOLATION`: Query failed content safety checks
- `RAG_PROCESSING_ERROR`: Error during vector search or document retrieval
- `AGENT_ERROR`: Error with OpenAI Agent processing
- `INTERNAL_ERROR`: General server error

## Webhook Events (Optional)

For real-time updates, the system may support webhooks:

#### POST /api/webhooks/chat
**Description**: Receive real-time chat events

**Payload**:
```json
{
  "event": "message.created",
  "threadId": "thread_abc123",
  "messageId": "msg_123",
  "timestamp": "2025-12-09T12:00:00Z"
}
```
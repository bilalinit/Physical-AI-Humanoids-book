# Chat History API Contract

## Overview
This document defines the API contracts for chat history persistence functionality, including endpoints for creating, retrieving, and managing chat threads and messages.

## Base URL
`/api/v1/chat`

## Endpoints

### 1. Create New Chat Thread
**POST** `/threads`

Creates a new chat thread for the authenticated user.

#### Request
- **Headers**:
  - `Authorization: Bearer <JWT_TOKEN>` (required)
  - `Content-Type: application/json`

- **Body**:
```json
{
  "title": "Optional custom title for the thread"
}
```

#### Response
- **Success (201 Created)**:
```json
{
  "id": "thread-uuid",
  "user_id": "user-uuid",
  "title": "Thread title",
  "created_at": "2025-12-16T10:30:00Z",
  "updated_at": "2025-12-16T10:30:00Z"
}
```

- **Errors**:
  - 401: Unauthorized (invalid or missing JWT)
  - 400: Bad Request (invalid input data)
  - 500: Internal Server Error (database connection issues)

### 2. Get User's Chat Threads
**GET** `/threads`

Retrieves all chat threads for the authenticated user.

#### Request
- **Headers**:
  - `Authorization: Bearer <JWT_TOKEN>` (required)
  - `Limit: <number>` (optional, default: 50)
  - `Offset: <number>` (optional, default: 0)

#### Response
- **Success (200 OK)**:
```json
{
  "threads": [
    {
      "id": "thread-uuid-1",
      "title": "First conversation",
      "created_at": "2025-12-15T10:30:00Z",
      "updated_at": "2025-12-16T09:15:00Z"
    },
    {
      "id": "thread-uuid-2",
      "title": "Second conversation",
      "created_at": "2025-12-14T14:20:00Z",
      "updated_at": "2025-12-14T14:25:00Z"
    }
  ],
  "total_count": 2,
  "limit": 50,
  "offset": 0
}
```

- **Errors**:
  - 401: Unauthorized (invalid or missing JWT)
  - 500: Internal Server Error (database connection issues)

### 3. Get Chat Thread Details
**GET** `/threads/{thread_id}`

Retrieves details of a specific chat thread and its messages.

#### Request
- **Headers**:
  - `Authorization: Bearer <JWT_TOKEN>` (required)
- **Path Parameters**:
  - `thread_id` (string, required): ID of the chat thread

#### Response
- **Success (200 OK)**:
```json
{
  "id": "thread-uuid",
  "title": "Conversation title",
  "created_at": "2025-12-15T10:30:00Z",
  "updated_at": "2025-12-16T09:15:00Z",
  "messages": [
    {
      "id": "message-uuid-1",
      "role": "user",
      "content": "Hello, how can I use this?",
      "sources": [],
      "selected_text_used": false,
      "created_at": "2025-12-15T10:30:00Z"
    },
    {
      "id": "message-uuid-2",
      "role": "assistant",
      "content": "You can use this by following these steps...",
      "sources": [
        {
          "type": "document",
          "id": "doc-uuid",
          "title": "User Guide",
          "section": "Getting Started"
        }
      ],
      "selected_text_used": true,
      "created_at": "2025-12-15T10:31:00Z"
    }
  ]
}
```

- **Errors**:
  - 401: Unauthorized (invalid or missing JWT)
  - 404: Not Found (thread doesn't exist or belongs to another user)
  - 500: Internal Server Error (database connection issues)

### 4. Add Message to Chat Thread
**POST** `/threads/{thread_id}/messages`

Adds a new message to an existing chat thread.

#### Request
- **Headers**:
  - `Authorization: Bearer <JWT_TOKEN>` (required)
  - `Content-Type: application/json`
- **Path Parameters**:
  - `thread_id` (string, required): ID of the chat thread
- **Body**:
```json
{
  "role": "user",
  "content": "This is the message content",
  "sources": [
    {
      "type": "document",
      "id": "doc-uuid",
      "title": "Document Title",
      "section": "Section Name"
    }
  ],
  "selected_text_used": true
}
```

#### Response
- **Success (201 Created)**:
```json
{
  "id": "message-uuid",
  "role": "user",
  "content": "This is the message content",
  "sources": [
    {
      "type": "document",
      "id": "doc-uuid",
      "title": "Document Title",
      "section": "Section Name"
    }
  ],
  "selected_text_used": true,
  "created_at": "2025-12-16T10:30:00Z"
}
```

- **Errors**:
  - 401: Unauthorized (invalid or missing JWT)
  - 404: Not Found (thread doesn't exist or belongs to another user)
  - 400: Bad Request (invalid input data)
  - 500: Internal Server Error (database connection issues)

### 5. Update Chat Thread Title
**PUT** `/threads/{thread_id}`

Updates the title of an existing chat thread.

#### Request
- **Headers**:
  - `Authorization: Bearer <JWT_TOKEN>` (required)
  - `Content-Type: application/json`
- **Path Parameters**:
  - `thread_id` (string, required): ID of the chat thread
- **Body**:
```json
{
  "title": "New conversation title"
}
```

#### Response
- **Success (200 OK)**:
```json
{
  "id": "thread-uuid",
  "title": "New conversation title",
  "updated_at": "2025-12-16T10:30:00Z"
}
```

- **Errors**:
  - 401: Unauthorized (invalid or missing JWT)
  - 404: Not Found (thread doesn't exist or belongs to another user)
  - 400: Bad Request (invalid input data)
  - 500: Internal Server Error (database connection issues)

### 6. Delete Chat Thread
**DELETE** `/threads/{thread_id}`

Deletes a chat thread and all its messages.

#### Request
- **Headers**:
  - `Authorization: Bearer <JWT_TOKEN>` (required)
- **Path Parameters**:
  - `thread_id` (string, required): ID of the chat thread

#### Response
- **Success (204 No Content)**: Thread successfully deleted

- **Errors**:
  - 401: Unauthorized (invalid or missing JWT)
  - 404: Not Found (thread doesn't exist or belongs to another user)
  - 500: Internal Server Error (database connection issues)

## Authentication
All endpoints (except health check) require a valid JWT token in the Authorization header. The token is validated against the auth server to ensure the user is authenticated.

## Error Response Format
All error responses follow this format:
```json
{
  "error": {
    "code": "ERROR_CODE",
    "message": "Human-readable error message",
    "details": "Optional additional details"
  }
}
```

## Common Error Codes
- `AUTHENTICATION_REQUIRED`: No valid authentication token provided
- `INSUFFICIENT_PERMISSIONS`: User doesn't have access to requested resource
- `RESOURCE_NOT_FOUND`: Requested resource doesn't exist
- `VALIDATION_ERROR`: Request data doesn't meet validation requirements
- `DATABASE_ERROR`: Issue with database connection or operation
- `INTERNAL_ERROR`: Unexpected server error
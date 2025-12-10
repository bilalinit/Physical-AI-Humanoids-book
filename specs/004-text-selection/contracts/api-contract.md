# API Contract: Text Selection Enhancement

## Overview
This contract defines the API changes needed to support the text selection to chat feature. The existing `/api/chat` endpoint will be enhanced to accept selected text as additional context.

## Enhanced Chat Endpoint

### POST /api/chat

#### Description
Enhanced chat endpoint that accepts selected text as additional context for AI responses.

#### Request Body
```json
{
  "user_query": {
    "type": "string",
    "description": "The user's query to the AI",
    "required": true,
    "minLength": 1,
    "maxLength": 10000
  },
  "selected_text": {
    "type": "string",
    "description": "Text selected by user from book content (optional)",
    "required": false,
    "maxLength": 5000
  },
  "chat_history": {
    "type": "array",
    "description": "Previous messages in the conversation",
    "required": false,
    "default": [],
    "items": {
      "type": "object",
      "properties": {
        "role": {
          "type": "string",
          "enum": ["user", "assistant"]
        },
        "content": {
          "type": "string"
        }
      }
    }
  },
  "user_id": {
    "type": "string",
    "description": "Authenticated user ID (optional)",
    "required": false
  }
}
```

#### Response Body
```json
{
  "output": {
    "type": "string",
    "description": "AI response text"
  },
  "context_chunks": {
    "type": "array",
    "description": "Relevant content chunks from RAG search",
    "items": {
      "type": "object",
      "properties": {
        "content": "string",
        "source": "string",
        "score": "number"
      }
    }
  },
  "sources": {
    "type": "array",
    "description": "Sources referenced in the response",
    "items": {
      "type": "string"
    }
  },
  "used_selected_text": {
    "type": "boolean",
    "description": "Whether the selected text was used in generating the response"
  }
}
```

#### Error Responses
- `400 Bad Request`: Invalid request format
- `401 Unauthorized`: Invalid or missing authentication
- `500 Internal Server Error`: AI service unavailable

#### Security
- Requires valid JWT token in Authorization header
- Input validation for all fields to prevent injection attacks

## Frontend-to-Backend Communication

### Text Selection Event API
The frontend will use the existing chat API but with the enhanced request format that includes selected_text.

### Authentication Flow
1. Frontend sends JWT token with each request
2. Backend validates token against auth server
3. Request is processed with user profile data included in context

## Backward Compatibility
- The `selected_text` field is optional, ensuring existing chat functionality continues to work
- All existing request/response fields remain unchanged
- Default behavior is preserved when `selected_text` is null or not provided
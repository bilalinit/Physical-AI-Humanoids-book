# API Contracts: Docusaurus Book for Physical AI & Humanoid Robotics

## Authentication API

### POST /api/auth/register
Register a new user
- Request:
  - email: string (required)
  - password: string (required, min 8 chars)
  - name: string (required)
  - profile: object (optional)
    - education: string
    - software_experience: string
    - hardware_experience: string
    - robotics_background: string

- Response (201):
  - user_id: string
  - email: string
  - name: string
  - profile: object

- Response (400): Validation error
- Response (409): User already exists

### POST /api/auth/login
Login existing user
- Request:
  - email: string (required)
  - password: string (required)

- Response (200):
  - token: string
  - user: object
    - id: string
    - email: string
    - name: string

- Response (401): Invalid credentials

### GET /api/auth/profile
Get current user profile
- Headers:
  - Authorization: Bearer {token}

- Response (200):
  - user: object
    - id: string
    - email: string
    - name: string
    - profile: object

- Response (401): Invalid token

## Documentation API

### GET /api/docs/chapters
Get list of all documentation chapters
- Query params:
  - part: string (optional) - Filter by part
  - limit: integer (optional) - Number of results to return
  - offset: integer (optional) - Number of results to skip

- Response (200):
  - chapters: array of objects
    - id: string
    - title: string
    - slug: string
    - part: string
    - chapter_number: integer
    - learning_objectives: array of strings

### GET /api/docs/chapters/{slug}
Get specific chapter content
- Path params:
  - slug: string - Chapter slug

- Response (200):
  - id: string
  - title: string
  - slug: string
  - part: string
  - chapter_number: integer
  - content: string (Markdown)
  - prerequisites: array of strings
  - learning_objectives: array of strings
  - code_examples: array of objects
  - diagrams: array of strings
  - related_topics: array of strings

- Response (404): Chapter not found

### GET /api/docs/code-examples
Get code examples for a chapter
- Query params:
  - chapter_id: string (required)
  - language: string (optional) - Filter by programming language

- Response (200):
  - examples: array of objects
    - id: string
    - title: string
    - language: string
    - code: string
    - description: string
    - file_path: string
    - dependencies: array of strings
    - execution_notes: string

## Search API

### POST /api/search
Search documentation content
- Request:
  - query: string (required)
  - filters: object (optional)
    - parts: array of strings
    - chapter_numbers: array of integers

- Response (200):
  - query: string
  - results: array of objects
    - chapter_id: string
    - chapter_title: string
    - snippet: string
    - score: float
    - url: string
  - total_results: integer
  - response_time: float

- Response (400): Invalid query

## Chat API

### POST /api/chat/sessions
Create a new chat session
- Headers:
  - Authorization: Bearer {token} (optional)
- Request:
  - title: string (optional)

- Response (201):
  - session_id: string
  - title: string
  - created_at: string (ISO date)

### GET /api/chat/sessions/{sessionId}
Get chat session details
- Headers:
  - Authorization: Bearer {token}
- Path params:
  - sessionId: string

- Response (200):
  - session: object
    - id: string
    - title: string
    - created_at: string (ISO date)
    - updated_at: string (ISO date)

### POST /api/chat/sessions/{sessionId}/messages
Send a message in a chat session
- Headers:
  - Authorization: Bearer {token}
- Path params:
  - sessionId: string
- Request:
  - content: string (required)

- Response (201):
  - message: object
    - id: string
    - content: string
    - sender: "user"
    - timestamp: string (ISO date)
  - response: object
    - id: string
    - content: string
    - sender: "assistant"
    - timestamp: string (ISO date)
    - context_chunks: array of strings

### GET /api/chat/sessions/{sessionId}/messages
Get messages from a chat session
- Headers:
  - Authorization: Bearer {token}
- Path params:
  - sessionId: string
- Query params:
  - limit: integer (optional)
  - offset: integer (optional)

- Response (200):
  - messages: array of objects
    - id: string
    - content: string
    - sender: string ("user" or "assistant")
    - timestamp: string (ISO date)

## Learning Path API

### GET /api/paths
Get available learning paths
- Response (200):
  - paths: array of objects
    - id: string
    - name: string
    - description: string
    - target_audience: string
    - estimated_duration: string
    - chapter_count: integer

### GET /api/paths/{pathId}
Get details of a specific learning path
- Path params:
  - pathId: string

- Response (200):
  - id: string
  - name: string
  - description: string
  - target_audience: string
  - prerequisites: array of strings
  - estimated_duration: string
  - chapters: array of objects
    - chapter_id: string
    - title: string
    - required: boolean
# Data Model: Chat History Database Persistence

## Overview
This document defines the data models for the chat history persistence feature, including entities, relationships, and validation rules.

## Entities

### ChatThread
Represents a conversation session that contains multiple messages.

**Fields:**
- `id` (UUID/String): Unique identifier for the chat thread
- `user_id` (String): Foreign key reference to the user who owns this thread
- `title` (String): Display title for the chat thread (auto-generated from first message or customizable)
- `created_at` (DateTime): Timestamp when the thread was created
- `updated_at` (DateTime): Timestamp when the thread was last updated

**Relationships:**
- One-to-Many: Contains multiple ChatMessage entities
- Many-to-One: Belongs to a single User entity

**Validation Rules:**
- `user_id` must be a valid authenticated user ID
- `title` must be between 1 and 200 characters
- `created_at` is set automatically on creation
- `updated_at` is updated automatically when new messages are added

### ChatMessage
Represents an individual message in a conversation.

**Fields:**
- `id` (UUID/String): Unique identifier for the message
- `thread_id` (String): Foreign key reference to the parent chat thread
- `role` (String): Message sender role ('user' or 'assistant')
- `content` (Text): The actual message content
- `sources` (JSONB): Document sources referenced in the message (if any)
- `selected_text_used` (Boolean): Whether selected text was used in this message
- `created_at` (DateTime): Timestamp when the message was created

**Relationships:**
- Many-to-One: Belongs to a single ChatThread entity
- Many-to-One: Associated with the user through the thread

**Validation Rules:**
- `thread_id` must reference an existing chat thread
- `role` must be either 'user' or 'assistant'
- `content` must not be empty
- `created_at` is set automatically on creation
- `sources` must be valid JSON if provided

### User (Reference)
Represents an authenticated user in the system.

**Fields:**
- `id` (String/UUID): Unique identifier for the user
- `email` (String): User's email address
- `name` (String): User's display name
- `created_at` (DateTime): When the user account was created

**Relationships:**
- One-to-Many: Owns multiple ChatThread entities

## Database Schema

### chat_threads table
```sql
CREATE TABLE chat_threads (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id VARCHAR(255) NOT NULL,
    title VARCHAR(200) NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE
);

CREATE INDEX idx_chat_threads_user_id ON chat_threads(user_id);
CREATE INDEX idx_chat_threads_updated_at ON chat_threads(updated_at);
```

### chat_messages table
```sql
CREATE TABLE chat_messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    thread_id UUID NOT NULL,
    role VARCHAR(20) NOT NULL,
    content TEXT NOT NULL,
    sources JSONB,
    selected_text_used BOOLEAN DEFAULT FALSE,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    FOREIGN KEY (thread_id) REFERENCES chat_threads(id) ON DELETE CASCADE
);

CREATE INDEX idx_chat_messages_thread_id ON chat_messages(thread_id);
CREATE INDEX idx_chat_messages_created_at ON chat_messages(created_at);
```

## State Transitions

### ChatThread States
- **Active**: Thread is being used for current conversation
- **Archived**: Thread is preserved but not actively used (future feature)

### ChatMessage States
- **Created**: Message is stored in the database
- **Updated**: Message content is modified (future feature)
- **Deleted**: Message is removed (future feature with soft delete)

## Constraints and Business Rules

1. **User Access Control**: Users can only access threads associated with their user ID
2. **Cascade Deletion**: When a user is deleted, all their chat threads and messages are automatically removed
3. **Thread Timestamp Update**: When a new message is added to a thread, the thread's `updated_at` timestamp is automatically updated
4. **Message Ordering**: Messages within a thread are ordered by `created_at` timestamp
5. **Data Integrity**: Foreign key constraints ensure referential integrity between entities
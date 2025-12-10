# Data Model: ChatKit Integration

## Entities

### Chat Thread
**Description**: Represents a conversation session with message history and context
**Fields**:
- `id` (string): Unique identifier for the thread
- `createdAt` (timestamp): When the thread was created
- `updatedAt` (timestamp): When the thread was last updated
- `userId` (string): ID of the user who owns the thread (optional for anonymous users)
- `messages` (array): List of messages in the thread
- `metadata` (object): Additional thread-specific data

**Relationships**:
- One-to-many with Messages
- Many-to-one with User (optional)

### Message
**Description**: Represents a single message in a conversation thread
**Fields**:
- `id` (string): Unique identifier for the message
- `threadId` (string): ID of the thread this message belongs to
- `role` (string): 'user' or 'assistant'
- `content` (string): The message content
- `timestamp` (timestamp): When the message was sent
- `metadata` (object): Additional message-specific data (source documents, etc.)

**Relationships**:
- Many-to-one with Chat Thread
- Belongs to a specific thread context

### User Session
**Description**: Represents authenticated user state and permissions
**Fields**:
- `id` (string): Unique user identifier
- `authToken` (string): JWT token for authentication
- `profile` (object): User profile data (education, experience, etc.)
- `permissions` (array): User permissions and access levels
- `createdAt` (timestamp): When the session was created

**Relationships**:
- One-to-many with Chat Threads (for authenticated users)

### RAG Query
**Description**: Represents a user question that triggers vector search and document retrieval
**Fields**:
- `queryText` (string): The original user query
- `queryVector` (array): Vector representation of the query
- `retrievedDocuments` (array): List of documents retrieved from Qdrant
- `context` (string): Contextual information for the LLM
- `timestamp` (timestamp): When the query was processed

**Relationships**:
- Associated with a Message (when RAG processing occurs)

### Agent Context
**Description**: Represents the memory state maintained by OpenAI Agents during conversation
**Fields**:
- `sessionId` (string): ID of the current session
- `conversationHistory` (array): Previous conversation turns
- `currentContext` (string): Current conversation context
- `memoryState` (object): Agent-specific memory data
- `lastInteraction` (timestamp): When context was last updated

**Relationships**:
- Associated with a Chat Thread
- Influences Message generation

## Validation Rules

### Chat Thread
- Must have a unique ID
- CreatedAt must be before updatedAt
- Messages must be in chronological order

### Message
- Must have a valid role ('user' or 'assistant')
- Must belong to an existing thread
- Content must not exceed token limits
- Timestamp must be within thread lifetime

### User Session
- AuthToken must be valid JWT
- Profile data must match expected schema
- Permissions must be from predefined set

### RAG Query
- Query text must not be empty
- Retrieved documents must exist in Qdrant
- Context must be properly formatted for LLM

### Agent Context
- SessionId must correspond to active session
- Memory state must be serializable
- Context must not exceed token limits

## State Transitions

### Chat Thread
1. **Created**: New thread initialized
2. **Active**: Messages being exchanged
3. **Inactive**: No recent activity
4. **Archived**: Thread completed and stored

### Message
1. **Pending**: Message sent, awaiting processing
2. **Processing**: RAG query being executed
3. **Generated**: Response created by LLM
4. **Delivered**: Message visible to user

### User Session
1. **Unauthenticated**: No user identity established
2. **Authenticating**: Authentication in progress
3. **Authenticated**: Valid session established
4. **Expired**: Session token expired
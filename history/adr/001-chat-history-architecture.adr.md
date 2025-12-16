# ADR 001: Chat History Persistence Architecture

## Status
Accepted

## Context
The existing chat history implementation used in-memory storage that was lost on server restarts. We needed to implement persistent storage while maintaining compatibility with the existing Better Auth system and following the project's architectural principles.

## Decision
We decided to implement a thread-based chat history system with the following architecture:

1. **Data Model**:
   - Use a two-table approach with `chat_threads` and `chat_messages` tables
   - Maintain foreign key relationship from messages to threads
   - Include user_id on threads for efficient user-based queries
   - Use UUID primary keys for better distributed systems compatibility
   - Use JSONB for flexible message sources storage

2. **Database Layer**:
   - Implement async SQLAlchemy operations for non-blocking I/O
   - Use repository pattern for clean separation of data access logic
   - Implement proper connection pooling and session management

3. **Service Layer**:
   - Create dedicated service classes for business logic
   - Maintain compatibility with existing Better Auth session validation
   - Add JWT validation utilities for future extensibility

4. **API Design**:
   - Follow REST conventions with proper HTTP status codes
   - Implement user isolation at both service and API layers
   - Provide comprehensive endpoints for thread and message operations

## Alternatives Considered

### Single Table Approach
- Store all messages with user_id and thread_id in a single table
- Rejected due to complexity in thread management and potential performance issues

### Document-Based Storage
- Use MongoDB or similar for chat history
- Rejected to maintain consistency with existing PostgreSQL infrastructure

### Message-First Design
- Store messages with thread metadata embedded
- Rejected in favor of proper relational structure for better querying

## Consequences

### Positive
- Better organization of chat conversations in threads
- Improved query performance with proper indexing
- Clear separation of concerns with repository pattern
- Maintained security with user isolation
- Flexible message metadata with JSONB

### Negative
- More complex schema than flat approach
- Additional join operations for thread + messages queries
- More complex migration from existing flat structure

## Implementation
The architecture was implemented with SQLAlchemy async models, repository pattern services, and comprehensive API endpoints following the v1 API design.
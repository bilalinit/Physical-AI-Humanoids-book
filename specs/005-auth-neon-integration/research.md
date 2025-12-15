# Research: Better Auth + Neon Integration for RAG Chatbot

**Feature**: 005-auth-neon-integration
**Date**: 2025-12-13
**Purpose**: Resolve technical unknowns and establish implementation patterns

## Research Tasks

### 1. Better Auth Integration with Existing JWT Backend

**Context**: Existing backend uses JWT authentication with `get_current_user()` and `get_current_user_optional()` functions. Need to integrate Better Auth session validation.

**Research Findings**:
- **Decision**: Keep existing JWT functions for backward compatibility, add Better Auth session validation as middleware
- **Rationale**:
  - Existing code already handles JWT tokens in Authorization header
  - Better Auth uses cookie-based sessions (`better-auth.session_token`)
  - Can validate sessions by calling auth server API: `GET /api/auth/get-session`
  - Return user data including UUID, email, and learning preferences
- **Alternatives considered**:
  - Replace JWT entirely with Better Auth: Would break existing anonymous user support
  - Dual authentication (JWT + Better Auth): More complex but maintains compatibility
- **Implementation Pattern**:
  ```python
  # In backend FastAPI middleware/dependency
  async def validate_better_auth_session(request: Request):
      session_token = request.cookies.get("better-auth.session_token")
      if not session_token:
          return None  # Anonymous user

      # Call auth server to validate session
      auth_response = await call_auth_server("/api/auth/get-session", session_token)
      if auth_response.status == 200:
          return auth_response.user_data  # {id, email, name, learning_preferences}
      return None  # Invalid session
  ```

### 2. Database Schema Design for Better Auth + Chat History

**Context**: Need to create PostgreSQL tables for Better Auth (user, session) and chat_history with proper relationships.

**Research Findings**:
- **Decision**: Use camelCase column names as per Better Auth convention, UUID strings for user IDs
- **Rationale**:
  - Better Auth expects specific table/column naming conventions
  - UUID strings (TEXT) ensure compatibility across services
  - Foreign key relationships must use `ON DELETE CASCADE` for data integrity
- **SQL Schema**:
  ```sql
  -- Better Auth tables (camelCase)
  CREATE TABLE "user" (
      "id" TEXT PRIMARY KEY,  -- UUID string
      "email" TEXT NOT NULL UNIQUE,
      "name" TEXT NOT NULL,
      "educationLevel" TEXT,
      "programmingExperience" TEXT,
      "softwareBackground" TEXT,
      "hardwareBackground" TEXT,
      "roboticsBackground" TEXT,
      "createdAt" TIMESTAMP DEFAULT NOW()
  );

  CREATE TABLE "session" (
      "id" TEXT PRIMARY KEY,
      "userId" TEXT REFERENCES "user"("id") ON DELETE CASCADE,
      "token" TEXT,
      "expiresAt" TIMESTAMP NOT NULL
  );

  -- Chat history table
  CREATE TABLE "chat_history" (
      "id" SERIAL PRIMARY KEY,
      "userId" TEXT NOT NULL REFERENCES "user"("id") ON DELETE CASCADE,
      "message" TEXT NOT NULL,
      "response" TEXT NOT NULL,
      "selectedText" TEXT,
      "createdAt" TIMESTAMP NOT NULL DEFAULT NOW()
  );

  -- Indexes for performance
  CREATE INDEX idx_chat_history_user_id ON "chat_history"("userId");
  CREATE INDEX idx_chat_history_created_at ON "chat_history"("createdAt" DESC);
  ```

### 3. CORS Configuration for Tri-Service Architecture

**Context**: Frontend (port 3000), Auth Server (port 3001), Backend (port 8000) need proper CORS configuration.

**Research Findings**:
- **Decision**: Configure CORS per service with credentials support
- **Rationale**:
  - Cookies (session tokens) require `credentials: 'include'` and proper CORS headers
  - Each service needs to accept requests from other services' origins
  - Development vs production origins differ
- **Implementation**:
  ```typescript
  // Auth Server CORS (Express)
  app.use(cors({
      origin: ['http://localhost:3000', 'https://your-production-frontend.com'],
      credentials: true,
      methods: ['GET', 'POST', 'PUT', 'DELETE', 'OPTIONS']
  }));

  // Backend CORS (FastAPI)
  app.add_middleware(
      CORSMiddleware,
      allow_origins=["http://localhost:3000", "http://localhost:3001", "https://your-production-frontend.com"],
      allow_credentials=True,
      allow_methods=["*"],
      allow_headers=["*"],
  )
  ```

### 4. Personalization Integration with OpenAI Agents SDK

**Context**: Need to inject user learning preferences into system prompts for personalized responses.

**Research Findings**:
- **Decision**: Modify system prompt based on user's education level and experience
- **Rationale**:
  - OpenAI Agents SDK allows dynamic system prompt configuration
  - User preferences (education, programming experience, robotics background) affect response complexity
  - Can create prompt templates for different user levels
- **Implementation Pattern**:
  ```python
  def get_personalized_system_prompt(user_profile: UserProfile) -> str:
      base_prompt = """You are a helpful AI assistant for a robotics and programming book..."""

      # Add personalization based on user level
      if user_profile.education_level == "High School":
          base_prompt += "\n\nIMPORTANT: The user is at high school level. Use simple language..."
      elif user_profile.programming_experience == "Beginner":
          base_prompt += "\n\nIMPORTANT: The user is a programming beginner. Explain concepts..."
      # ... more personalization rules

      return base_prompt
  ```

### 5. Database Connection Pooling for Neon PostgreSQL

**Context**: Multiple services (auth server, backend) need efficient database connections to Neon.

**Research Findings**:
- **Decision**: Use connection pooling with appropriate pool sizes per service
- **Rationale**:
  - Neon PostgreSQL supports connection pooling via `pgbouncer` or client-side pooling
  - Auth server needs fewer connections (user auth operations)
  - Backend needs more connections (chat history operations)
- **Implementation**:
  ```typescript
  // Auth Server PostgreSQL client
  const pool = new Pool({
      connectionString: process.env.DATABASE_URL,
      max: 10,  // Smaller pool for auth operations
      idleTimeoutMillis: 30000,
      connectionTimeoutMillis: 2000,
      ssl: { rejectUnauthorized: false }  // Required for Neon
  });
  ```

### 6. Migration Strategy from In-Memory to Persistent Storage

**Context**: Existing chat threads are stored in-memory. Need to migrate to PostgreSQL.

**Research Findings**:
- **Decision**: Dual-write during transition, then migrate old data
- **Rationale**:
  - Cannot lose existing in-memory threads during deployment
  - Gradual migration reduces risk
  - Backward compatibility during transition period
- **Migration Steps**:
  1. Implement PostgreSQL chat history storage alongside existing in-memory
  2. For authenticated users: save to both in-memory and PostgreSQL
  3. For anonymous users: keep in-memory only (no user ID)
  4. Gradually phase out in-memory storage as confidence grows
  5. Optional: migrate existing in-memory threads to PostgreSQL on user login

### 7. Error Handling and User Experience

**Context**: Need comprehensive error handling for auth failures, session expiry, database issues.

**Research Findings**:
- **Decision**: Structured error responses with user-friendly messages
- **Rationale**:
  - Users need clear feedback on auth failures
  - Session expiry should trigger automatic redirect to login
  - Database errors should not expose internal details
- **Error Categories**:
  - **Auth Errors**: Invalid credentials, email already exists, password requirements
  - **Session Errors**: Expired session, invalid token
  - **Database Errors**: Connection failures, constraint violations
  - **Network Errors**: Service unavailable, timeout

## Key Technical Decisions Summary

1. **Authentication Strategy**: Dual-mode (JWT + Better Auth) for backward compatibility
2. **Database Schema**: camelCase with UUID strings, proper indexes, CASCADE deletes
3. **CORS Configuration**: Per-service with credentials support for cookie-based sessions
4. **Personalization**: Dynamic system prompts based on user learning preferences
5. **Connection Pooling**: Service-appropriate pool sizes for Neon PostgreSQL
6. **Migration Approach**: Gradual transition with dual-write strategy
7. **Error Handling**: Structured errors with user-friendly messages

## Open Questions Resolved

- ✅ **User ID format**: UUID strings (TEXT) not integers
- ✅ **Column naming**: camelCase for Better Auth compatibility
- ✅ **Session validation**: API call to auth server vs direct database query
- ✅ **Personalization method**: System prompt modification vs separate models
- ✅ **Migration strategy**: Gradual transition vs big-bang migration
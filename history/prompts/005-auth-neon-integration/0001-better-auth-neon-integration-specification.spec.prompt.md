---
id: 0001
title: Better Auth Neon Integration Specification
stage: spec
date: 2025-12-13
surface: agent
model: claude-sonnet-4-5-20250929
feature: 005-auth-neon-integration
branch: 005-auth-neon-integration
user: unknown
command: /sp.specify
labels: ["authentication", "database", "rag", "chat-history"]
links:
  spec: /mnt/d/coding Q4/main-hackathon/save-7/hackathon-book/specs/005-auth-neon-integration/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - /mnt/d/coding Q4/main-hackathon/save-7/hackathon-book/specs/005-auth-neon-integration/spec.md
  - /mnt/d/coding Q4/main-hackathon/save-7/hackathon-book/specs/005-auth-neon-integration/checklists/requirements.md
tests:
  - none
---

## Prompt

lets move on to  the next phase, name the new branch "005-auth-neon-integration" : # RAG Chatbot Enhancement: Better Auth & Neon Integration Plan

> **Module:** Secure User Authentication via Better Auth and Persistent RAG History in Neon PostgreSQL  
> **Tech Stack:** ChatKit, OpenAI Agents SDK, Qdrant DB, Gemini LLM/Embeddings, Docusaurus  
> **Date:** December 13, 2025

---

## System Upgrade Summary

This update expands your **current RAG chatbot infrastructure** by introducing:

1.  **Secure Login** using Better Auth (Email/Password, Self-Hosted)
2.  **Centralized Account Management** - User data stored directly in Neon PostgreSQL
3.  **Customized User Profiles** tracking learning preferences (e.g., education, coding exp)
4.  **Persistent Conversation Logs** - Archiving queries and AI replies to Neon
5.  **Context-Aware AI Responses** adapted to specific user profile data

> **Unified Data in Neon:** User identities, active sessions, and RAG conversation logs are now consolidated in a single PostgreSQL database.

### Current System Capabilities (Pre-integration)

Your existing RAG setup currently features:
- ✅ **ChatKit** handling the UI components
- ✅ **OpenAI Agents SDK** managing guardrails (driven by **Gemini LLM**)
- ✅ **Gemini Embeddings** handling vectorization
- ✅ **Qdrant** serving as the vector search engine for book content
- ✅ Contextual text selection capabilities
- ✅ Transient session history (clears on refresh)

### New Capabilities via this Update

- 🆕 **Permanent archiving** of all RAG interactions in Neon
- 🆕 **User-specific history** linked to individual accounts
- 🆕 **Session restoration** allowing chats to survive browser refreshes
- 🆕 **Context preservation** saving selected text alongside messages
- 🆕 **Adaptive complexity** based on the user's saved profile

---

## ⚠️ CRITICAL: Align with Existing Architecture

> [!IMPORTANT]
> **You must audit the project structure before writing code.**
> 
> 1. Scan your current file hierarchy and organization
> 2. Pinpoint the locations of current backend logic, frontend pages, and DB configs
> 3. Tailor the implementation to align with YOUR specific setup
> 4. Do NOT presume standard paths; verify them first

---

## Implementation Steps

### 🎯 Step 1: Inspect Project & Initialize Auth Service


Begin by auditing the current directory layout to understand the ecosystem.

Then, establish a Node.js authentication service using Better Auth backed by Neon PostgreSQL:

1. INSPECT: Review the project folders to locate:
   - Backend logic (Python/FastAPI endpoints)
   - Frontend source (Docusaurus/React files)
   - Any pre-existing auth configurations

2. INITIALIZE auth service (place appropriately within the project):
   - Implement the Better Auth library
   - Link to Neon PostgreSQL using DATABASE_URL
   - Activate Email/Password login strategy
   - Extend user schema with profile fields: softwareBackground, hardwareBackground, educationLevel, programmingExperience, roboticsBackground
   - Configure CORS to accept requests from the frontend origin

3. SETUP Express endpoints:
   - /api/auth/* - Handler for Better Auth routes
   - /api/user/me - Endpoint to fetch current session data
   - /api/user/update - Endpoint to modify user profile details

4. DEFINE schema migrations for Better Auth:
   - "user" table: TEXT id (UUID), email, name, and profile columns
   - "session" table: Must reference userId
   - "account" table: For future OAuth handling
   - Enforce camelCase naming for all columns

Ensure all imports and file paths correspond to MY actual project structure.


---

### 🎯 Step 2: Define Chat History Schema in Neon


Draft a migration to store RAG conversation logs in Neon PostgreSQL.

1. LOCATE: Identify where schema definitions or migration scripts are currently stored.

2. DEFINE table schema for "chat_history":
   - id: SERIAL PRIMARY KEY
   - userId: TEXT NOT NULL (Must match Better Auth "user" table id)
   - message: TEXT NOT NULL (The user's prompt)
   - response: TEXT NOT NULL (The AI's output)
   - selectedText: TEXT (Optional field for highlighted context)
   - createdAt: TIMESTAMP NOT NULL DEFAULT NOW()

3. OPTIMIZE with indexes:
   - Add index on userId for efficient retrieval
   - Add index on createdAt DESC for proper sorting

4. BUILD a migration runner:
   - Script should connect to Neon using SSL
   - Run the SQL command
   - Output success or error logs

IMPORTANT: userId must be TEXT (UUID string), not an integer, to match Better Auth.
Maintain camelCase column naming to align with the Auth schema.


---

### 🎯 Step 3: Implement Backend Persistence Utilities


Develop backend functions to handle the saving and loading of chat logs.

1. REVIEW: Examine the existing backend code handling:
   - RAG execution (OpenAI Agents, Qdrant, Gemini)
   - Database connections
   - The specific endpoint processing user chats

2. DEVELOP database helpers:
   
   get_chat_history(user_id: str, limit: int = 50):
   - Select from "chat_history" in Neon
   - Filter by "userId"
   - Sort by "createdAt" DESC
   - Return object list: id, message, response, selectedText, createdAt
   
   save_chat_message(user_id: str, message: str, response: str, selected_text: str = None):
   - Insert record into "chat_history"
   - Map to columns: "userId", "selectedText", "createdAt"
   - Return the generated chat ID

3. CONNECT to RAG logic:
   - Immediately after generating the AI response, trigger save_chat_message()
   - Inject the user_id from the active session
   - Capture selected_text if available in the request

Integrate seamlessly with current code patterns and style.


---

### 🎯 Step 4: Secure RAG Endpoints


Implement session verification to protect the chat interface.

1. EXAMINE: Look at the backend code containing:
   - The primary RAG endpoint
   - Any existing middleware layers

2. IMPLEMENT token verification:
   - Parse the "better-auth.session_token" cookie
   - Verify validity via the auth server: GET /api/auth/get-session
   - Retrieve user details (id, email, profile data)
   - Return 401 Unauthorized if validation fails

3. MODIFY chat route:
   - Enforce authentication check
   - Pass extracted user_id and profile data to the OpenAI Agents SDK
   - Use profile data to personalize the agent's behavior
   - Persist the interaction to the DB using the user_id

4. ADD /api/chat/history endpoint:
   - Protected route
   - Fetches and returns historical chats for the logged-in user

Do NOT disrupt the core RAG pipeline (Qdrant/Gemini/Guardrails must remain functional).


---

### 🎯 Step 5: Frontend Auth & UI Components


Integrate full authentication interfaces into the Docusaurus client.

1. ASSESS: Review the frontend architecture:
   - React component locations
   - The ChatKit/ChatBot implementation
   - Routing configuration
   - CSS/Theme definitions

2. UPDATE Chat Interface - Auth Guard:
   If User is Unauthenticated:
   - Hide the ChatKit interface
   - Display a "Login to Chat" button in its place
   - Button should feature a chat icon and clear call-to-action
   - On click: Redirect to /auth
   - Style: Match existing widget aesthetics
   
   If User is Authenticated:
   - Render the standard ChatKit interface
   - Load and populate previous chat history
   - (Optional) Show user avatar/name in the header

3. BUILD Login/Signup View (/auth):
   - Modern, clean form design
   - Toggle between "Sign In" and "Sign Up" states
   
   Sign In State:
   - Email & Password fields
   - Submit button
   - Toggle link to Sign Up
   
   Sign Up State:
   - Name, Email, Password, Confirm Password fields
   
   Inline Learning Preferences (during signup):
   - Education Level: High School | Undergraduate | Graduate | Professional
   - Programming Exp: No Experience | Beginner | Intermediate | Advanced
   - Robotics Background: No Experience | Hobbyist | Academic | Professional
   (Use selectable pill/chip buttons)
   
   - Submit button
   - Toggle link to Sign In
   
   Post-Signup Action:
   - Auto-redirect to home or Chat interface

4. DESIGN User Preferences Page (/profile):
   (Allows updates to learning settings)
   
   Header: "Learning Preferences"
   Subtitle: "Customize how the AI explains concepts to you."
   
   - Pre-load existing user choices
   
   Section 1 - Education:
   - "Adjusts explanation complexity"
   - Selectable Buttons (Single choice)
   
   Section 2 - Programming:
   - "Adjusts code example difficulty"
   - Selectable Buttons (Single choice)
   
   Section 3 - Robotics:
   - "Adjusts hardware explanation depth"
   - Selectable Buttons (Single choice)
   
   Footer: "Save Changes" button
   - POST to /api/user/update
   - Toast notification on success

5. MANAGE Error States:

   Auth Validation:
   - Bad email, Short password, Mismatching passwords
   - "Account already exists"
   - "Invalid credentials"
   
   Session Issues:
   - "Session expired" -> Redirect to login
   - "Unauthorized" -> Redirect to auth
   
   General UI:
   - Show loaders during API requests
   - Disable submit buttons while processing
   - Use red text for errors, green for success toasts
   - Clear errors on user input

6. VISUAL Design:
   - Adhere to the project's dark mode/glassmorphism theme
   - Pill buttons: Filled when active, Outline when inactive
   - Ensure mobile responsiveness

Use `credentials: 'include'` for all API fetch calls to handle cookies correctly.


---

### 🎯 Step 6: Validate Full Workflow


Execute a comprehensive test of the Auth + Persistence integration.

1. CHECK Schema Status:
   - execute migrations
   - confirm tables: "user", "session", "chat_history" exist

2. LAUNCH application stack:
   - Run Auth server, Backend, and Frontend

3. VALIDATE Login Process:
   - Create a new account
   - Confirm row creation in Neon "user" table
   - Log in and verify session cookie presence

4. TEST RAG Persistence:
   - Send a query to the bot
   - Confirm standard RAG response (Qdrant/Gemini)
   - SQL Query: SELECT * FROM "chat_history" to confirm save
   - Refresh browser -> Chat history should reappear

5. TEST Personalization:
   - Set profile to "Beginner" -> Ask technical question
   - Change profile to "Advanced" -> Ask same question
   - Analyze difference in response depth

6. TEST Context Feature:
   - Highlight text in docs
   - Query based on selection
   - Verify "selectedText" column is populated in DB


---

## SQL Schema Specifications

### Auth Tables (Neon PostgreSQL)


-- Using camelCase and TEXT IDs as per Better Auth standards
CREATE TABLE "user" (
    "id" TEXT PRIMARY KEY,           -- UUID string
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


### Chat Logs Table (Neon PostgreSQL)


CREATE TABLE "chat_history" (
    "id" SERIAL PRIMARY KEY,
    "userId" TEXT NOT NULL REFERENCES "user"("id") ON DELETE CASCADE,
    "message" TEXT NOT NULL,      -- User query
    "response" TEXT NOT NULL,     -- AI generation
    "selectedText" TEXT,          -- Contextual selection (nullable)
    "createdAt" TIMESTAMP NOT NULL DEFAULT NOW()
);


---

## Configuration Variables


# Database Connection (Shared)
DATABASE_URL=postgresql://user:password@host/database?sslmode=require

# RAG Configuration (Existing)
GEMINI_API_KEY=your_gemini_key
QDRANT_URL=https://your-cluster.qdrant.cloud
QDRANT_API_KEY=your_qdrant_key
OPENAI_API_KEY=your_openai_key


---

## Critical Implementation Details

1.  **ID Format:** User IDs are **TEXT** (UUIDs), not Integers.
2.  **Naming Convention:** Enforce **camelCase** for database columns.
3.  **Security:** Neon connections require `sslmode=require`.
4.  **Cookies:** Frontend requests must include `credentials: 'include'`.
5.  **Stability:** Do not alter the logic of existing Qdrant/Gemini integration.

## Response snapshot

Created feature specification for Better Auth + Neon integration with 4 prioritized user stories, 12 functional requirements, 6 success criteria, and comprehensive edge cases. Specification follows template structure and includes quality checklist validation.

## Outcome

- ✅ Impact: Foundation for user authentication and persistent chat history established
- 🧪 Tests: All acceptance scenarios defined for independent testing
- 📁 Files: spec.md and requirements checklist created
- 🔁 Next prompts: Ready for /sp.clarify or /sp.plan
- 🧠 Reflection: Specification adapts to existing project structure with JWT backend and ChatKit frontend

## Evaluation notes (flywheel)

- Failure modes observed: None - specification complete with no NEEDS CLARIFICATION markers
- Graders run and results (PASS/FAIL): All checklist items pass
- Prompt variant (if applicable): Standard /sp.specify
- Next experiment (smallest change to try): Proceed to planning phase

# Feature Specification: Better Auth + Neon Integration for RAG Chatbot

**Feature Branch**: `005-auth-neon-integration`
**Created**: 2025-12-13
**Status**: Draft
**Input**: User description: "lets move on to  the next phase, name the new branch "005-auth-neon-integration" : # Better Auth + Neon Integration for RAG Chatbot - Implementation Prompt

> **Feature:** User authentication with Better Auth and persistent RAG chat history using Neon PostgreSQL
> **Tech Stack:** ChatKit, OpenAI Agents SDK, Qdrant DB, Gemini LLM/Embeddings, Docusaurus
> **Created:** December 13, 2025

---

## Feature Overview

This integration adds to your **existing RAG chatbot system**:

1. **User Authentication** via Better Auth (email/password, self-hosted)
2. **User Accounts in Neon** - All users stored in Neon PostgreSQL
3. **User Profiles** with learning preferences (education level, programming experience)
4. **RAG Chat History Persistence** - Save user queries & AI responses to Neon
5. **Personalized AI Responses** based on user profile data

> **All data lives in Neon PostgreSQL:** Users, sessions, and RAG chat conversations stored in one database.

### Your Existing RAG System (Already Working)

Your RAG chatbot already has:
- ✅ **ChatKit** for chat UI components
- ✅ **OpenAI Agents SDK** with guardrails (using **Gemini LLM** via OpenAI-compatible API)
- ✅ **Gemini Embeddings** for query/document vectors
- ✅ **Qdrant** vector database for book content search
- ✅ Text selection context feature
- ✅ In-memory chat history during session

### What This Integration Adds

- 🆕 **Persistent storage** of all RAG conversations in Neon
- 🆕 **Per-user chat history** linked to authenticated users
- 🆕 **Chat reload on page refresh** - conversations survive browser close
- 🆕 **Selected text saved** with each chat message for context
- 🆕 **User profiles** for personalized AI response complexity"

## User Scenarios & Testing *(mandatory)*

<!--
  IMPORTANT: User stories should be PRIORITIZED as user journeys ordered by importance.
  Each user story/journey must be INDEPENDENTLY TESTABLE - meaning if you implement just ONE of them,
  you should still have a viable MVP (Minimum Viable Product) that delivers value.
  
  Assign priorities (P1, P2, P3, etc.) to each story, where P1 is the most critical.
  Think of each story as a standalone slice of functionality that can be:
  - Developed independently
  - Tested independently
  - Deployed independently
  - Demonstrated to users independently
-->

### User Story 1 - Create Account and Set Learning Preferences (Priority: P1)

A new user wants to create an account and set their learning preferences so they can get personalized AI responses that match their knowledge level.

**Why this priority**: Without user accounts, there's no way to personalize responses or persist chat history. This is the foundational capability that enables all other features.

**Independent Test**: Can be fully tested by creating a new user account, setting learning preferences, and verifying the preferences are saved and accessible.

**Acceptance Scenarios**:

1. **Given** a new user visiting the website, **When** they click "Sign Up" and complete the registration form with name, email, password, and learning preferences, **Then** their account is created, preferences are saved, and they are redirected to the chat interface.
2. **Given** a user with an existing account, **When** they visit the profile page, **Then** they see their current learning preferences and can update them.

---

### User Story 2 - Authenticate and Access Chat History (Priority: P2)

A returning user wants to sign in and see their previous chat conversations so they can continue learning from where they left off.

**Why this priority**: Chat history persistence is a key value-add over the current in-memory system. Users expect their conversations to survive browser refreshes and sessions.

**Independent Test**: Can be fully tested by signing in with an existing account, sending a chat message, refreshing the page, and verifying the chat history loads.

**Acceptance Scenarios**:

1. **Given** a registered user, **When** they sign in with correct credentials, **Then** they are authenticated and see their chat history from previous sessions.
2. **Given** an authenticated user, **When** they send a new chat message, **Then** the message and AI response are saved to their chat history.
3. **Given** an authenticated user, **When** they refresh the page, **Then** their chat history is restored and they remain authenticated.

---

### User Story 3 - Personalized RAG Responses Based on Profile (Priority: P3)

An authenticated user wants AI responses tailored to their education level and experience so they get explanations at the right complexity.

**Why this priority**: Personalization enhances learning effectiveness by adapting explanations to the user's background, making the chatbot more valuable than generic responses.

**Independent Test**: Can be fully tested by setting different learning preferences, asking the same technical question, and verifying the responses adapt in complexity.

**Acceptance Scenarios**:

1. **Given** a user with "Beginner" programming experience, **When** they ask a technical question, **Then** the AI response uses simpler language and includes basic explanations.
2. **Given** a user with "Advanced" programming experience, **When** they ask the same technical question, **Then** the AI response uses more technical language and assumes prior knowledge.
3. **Given** a user with "High School" education level, **When** they ask a complex concept question, **Then** the AI response breaks it down into fundamental concepts.

---

### User Story 4 - Text Selection Context Persistence (Priority: P3)

A user wants their selected text from documentation to be saved with chat messages so they can reference context later.

**Why this priority**: This enhances the existing text selection feature by making the context persistent, allowing users to understand why certain responses were given.

**Independent Test**: Can be fully tested by selecting text, asking a question about it, and verifying the selected text is saved with the chat message.

**Acceptance Scenarios**:

1. **Given** an authenticated user, **When** they select text from documentation and ask a question about it, **Then** the selected text is saved with the chat message in their history.
2. **Given** a user viewing their chat history, **When** they see a message with saved selected text, **Then** they can view the original context that influenced the AI response.

### Edge Cases

- What happens when a user tries to sign up with an email that already exists?
- How does the system handle password reset requests?
- What happens when a user's session expires while they're chatting?
- How does the system handle database connection failures during chat saving?
- What happens when a user tries to access chat history without being authenticated?
- How does the system handle very long chat messages or selected text?
- What happens when a user updates their learning preferences mid-conversation?
- How does the system handle concurrent chat sessions from the same user?

## Requirements *(mandatory)*

<!--
  ACTION REQUIRED: The content in this section represents placeholders.
  Fill them out with the right functional requirements.
-->

### Functional Requirements

- **FR-001**: System MUST allow users to create accounts with email, password, name, and learning preferences
- **FR-002**: System MUST validate email format and enforce password strength requirements
- **FR-003**: Users MUST be able to sign in with email and password credentials
- **FR-004**: System MUST persist user profiles including education level, programming experience, and robotics background
- **FR-005**: System MUST save all RAG chat conversations with user ID, message, response, and optional selected text
- **FR-006**: System MUST retrieve and display chat history for authenticated users
- **FR-007**: System MUST adapt AI response complexity based on user's learning preferences
- **FR-008**: System MUST require authentication for accessing chat history and sending messages
- **FR-009**: System MUST handle session expiration and redirect unauthenticated users to login
- **FR-010**: System MUST preserve existing RAG functionality (Qdrant search, Gemini responses, guardrails)
- **FR-011**: Users MUST be able to update their learning preferences after account creation
- **FR-012**: System MUST use UUID strings for user IDs and maintain referential integrity between tables

### Key Entities *(include if feature involves data)*

- **User**: Represents an authenticated user of the system. Key attributes: unique ID (UUID), email, name, education level, programming experience, robotics background, creation timestamp.
- **Session**: Represents an active user session. Key attributes: unique ID (UUID), user ID reference, authentication token, expiration timestamp.
- **Chat History**: Represents a saved conversation between a user and the AI. Key attributes: unique ID, user ID reference, user message, AI response, optional selected text, creation timestamp.

## Success Criteria *(mandatory)*

<!--
  ACTION REQUIRED: Define measurable success criteria.
  These must be technology-agnostic and measurable.
-->

### Measurable Outcomes

- **SC-001**: Users can create an account and set learning preferences in under 3 minutes
- **SC-002**: Authenticated users can access their complete chat history from previous sessions
- **SC-003**: AI responses adapt measurably based on user's learning preferences (e.g., beginner vs advanced explanations differ in technical depth)
- **SC-004**: Chat conversations survive browser refresh and session restart with 100% accuracy
- **SC-005**: System maintains existing RAG functionality performance (response time within 10% of current baseline)
- **SC-006**: 95% of users successfully authenticate on first attempt with clear error messages for failures

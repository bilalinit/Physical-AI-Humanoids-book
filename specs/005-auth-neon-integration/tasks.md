---
description: "Task list for Better Auth + Neon Integration for RAG Chatbot"
---

# Tasks: 005-auth-neon-integration

**Input**: Design documents from `/specs/005-auth-neon-integration/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `auth-server/src/`, `frontend/src/` per plan.md structure
- Paths shown below follow the tri-service architecture from plan.md

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create project structure per implementation plan in plan.md
- [ ] T002 [P] Initialize backend Python project with FastAPI dependencies in backend/requirements.txt
- [ ] T003 [P] Initialize auth server Node.js project with Express and Better Auth dependencies in auth-server/package.json
- [ ] T004 [P] Initialize frontend React project with auth dependencies in frontend/package.json
- [ ] T005 [P] Configure environment variables for all three services in .env files
- [ ] T006 [P] Setup Neon PostgreSQL database connection configuration in backend/src/database.py

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T007 Setup Neon PostgreSQL database schema from data-model.md in backend/src/database.py
- [ ] T008 [P] Implement database migration framework for schema evolution in backend/src/database.py
- [ ] T009 [P] Create base SQLAlchemy models for User, Session, ChatHistory in backend/src/models/base.py
- [ ] T010 [P] Setup Better Auth configuration in auth-server/src/config/auth.js
- [ ] T011 [P] Implement CORS middleware for cross-service communication in auth-server/src/middleware/cors.js
- [ ] T012 [P] Setup API routing structure for all three services
- [ ] T013 [P] Configure error handling and logging infrastructure across all services
- [ ] T014 Setup environment configuration management with .env synchronization

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Create Account and Set Learning Preferences (Priority: P1) 🎯 MVP

**Goal**: Users can create accounts with learning preferences for personalized AI responses

**Independent Test**: Create a new user account, set learning preferences, verify preferences are saved and accessible

### Tests for User Story 1 (OPTIONAL - only if tests requested) ⚠️

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [ ] T015 [P] [US1] Contract test for /api/auth/signup endpoint in auth-server/tests/test_signup.js
- [ ] T016 [P] [US1] Contract test for /api/auth/signin endpoint in auth-server/tests/test_signin.js
- [ ] T017 [P] [US1] Integration test for user registration journey in auth-server/tests/integration/test_registration.js

### Implementation for User Story 1

- [ ] T018 [P] [US1] Create User model with learning preferences in backend/src/models/user.py
- [ ] T019 [P] [US1] Create Session model in backend/src/models/session.py
- [ ] T020 [US1] Implement Better Auth signup endpoint in auth-server/src/routes/auth/signup.js
- [ ] T021 [US1] Implement Better Auth signin endpoint in auth-server/src/routes/auth/signin.js
- [ ] T022 [US1] Implement signout endpoint in auth-server/src/routes/auth/signout.js
- [ ] T023 [US1] Create signup form component in frontend/src/components/Auth/SignupForm.js
- [ ] T024 [US1] Create signin form component in frontend/src/components/Auth/SigninForm.js
- [ ] T025 [US1] Implement auth API client in frontend/src/services/auth.js
- [ ] T026 [US1] Add validation and error handling for auth forms in frontend/src/components/Auth/validation.js
- [ ] T027 [US1] Add logging for user registration operations in auth-server/src/middleware/logger.js

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Authenticate and Access Chat History (Priority: P2)

**Goal**: Returning users can sign in and see their previous chat conversations

**Independent Test**: Sign in with existing account, send chat message, refresh page, verify chat history loads

### Tests for User Story 2 (OPTIONAL - only if tests requested) ⚠️

- [ ] T028 [P] [US2] Contract test for /api/auth/get-session endpoint in auth-server/tests/test_session.js
- [ ] T029 [P] [US2] Contract test for /api/chat/history endpoint in backend/tests/test_chat_history.py
- [ ] T030 [P] [US2] Integration test for chat history persistence in backend/tests/integration/test_chat_persistence.py

### Implementation for User Story 2

- [ ] T031 [P] [US2] Create ChatHistory model in backend/src/models/chat_history.py
- [ ] T032 [US2] Implement session validation endpoint in auth-server/src/routes/auth/get-session.js
- [ ] T033 [US2] Implement session validation service in backend/src/services/auth_validation.py
- [ ] T034 [US2] Implement chat history save endpoint in backend/src/api/chat_history.py
- [ ] T035 [US2] Implement chat history retrieval endpoint in backend/src/api/chat_history.py
- [ ] T036 [US2] Create chat history service in backend/src/services/chat_history_service.py
- [ ] T037 [US2] Modify existing chat endpoint to save history in backend/src/api/chat.py
- [ ] T038 [US2] Implement useAuth hook for auth state management in frontend/src/hooks/useAuth.js
- [ ] T039 [US2] Create chat history component in frontend/src/components/Chat/ChatHistory.js
- [ ] T040 [US2] Integrate auth headers into chat API client in frontend/src/services/chat.js
- [ ] T041 [US2] Add session persistence on page refresh in frontend/src/hooks/useAuth.js

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Personalized RAG Responses Based on Profile (Priority: P3)

**Goal**: AI responses adapt to user's education level and experience for appropriate complexity

**Independent Test**: Set different learning preferences, ask same technical question, verify responses adapt in complexity

### Tests for User Story 3 (OPTIONAL - only if tests requested) ⚠️

- [ ] T042 [P] [US3] Contract test for /api/user/me endpoint in auth-server/tests/test_profile.js
- [ ] T043 [P] [US3] Integration test for personalized responses in backend/tests/integration/test_personalization.py

### Implementation for User Story 3

- [ ] T044 [P] [US3] Create user profile endpoint in auth-server/src/routes/user/me.js
- [ ] T045 [US3] Implement user profile update endpoint in auth-server/src/routes/user/me.js
- [ ] T046 [US3] Create profile component in frontend/src/components/Auth/Profile.js
- [ ] T047 [US3] Modify system prompt generation to include user preferences in backend/src/services/chat_service.py
- [ ] T048 [US3] Implement prompt personalization service in backend/src/services/prompt_personalization.py
- [ ] T049 [US3] Add user context injection into RAG pipeline in backend/src/services/rag_service.py
- [ ] T050 [US3] Create learning preferences form in frontend/src/components/Auth/LearningPreferencesForm.js
- [ ] T051 [US3] Add profile link to navigation in frontend/src/components/Navigation/Navbar.js

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: User Story 4 - Text Selection Context Persistence (Priority: P3)

**Goal**: Selected text from documentation is saved with chat messages for context reference

**Independent Test**: Select text, ask question about it, verify selected text is saved with chat message

### Tests for User Story 4 (OPTIONAL - only if tests requested) ⚠️

- [ ] T052 [P] [US4] Contract test for /api/chat endpoint with selectedText in backend/tests/test_chat_selection.py
- [ ] T053 [P] [US4] Integration test for text selection persistence in backend/tests/integration/test_text_selection.py

### Implementation for User Story 4

- [ ] T054 [P] [US4] Add selectedText field to ChatHistory model in backend/src/models/chat_history.py
- [ ] T055 [US4] Modify chat endpoint to accept and save selectedText in backend/src/api/chat.py
- [ ] T056 [US4] Update chat history retrieval to include selectedText in backend/src/api/chat_history.py
- [ ] T057 [US4] Modify chat request schema to include selectedText in backend/src/schemas/chat.py
- [ ] T058 [US4] Enhance chat UI to capture and display selected text in frontend/src/components/Chat/ChatInput.js
- [ ] T059 [US4] Add selected text display in chat history component in frontend/src/components/Chat/ChatHistory.js
- [ ] T060 [US4] Update database schema to include selectedText column in backend/src/database.py

**Checkpoint**: All four user stories should now be independently functional

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T061 [P] Documentation updates in docs/ for auth and chat history features
- [ ] T062 Code cleanup and refactoring across all three services
- [ ] T063 Performance optimization for chat history queries in backend/src/services/chat_history_service.py
- [ ] T064 [P] Additional unit tests for edge cases in all test directories
- [ ] T065 Security hardening for session management in auth-server/src/middleware/auth.js
- [ ] T066 Implement rate limiting for auth endpoints in auth-server/src/middleware/rateLimit.js
- [ ] T067 Add health check endpoints for all services
- [ ] T068 Setup monitoring and logging for production deployment
- [ ] T069 Run quickstart.md validation for end-to-end testing

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Depends on US1 for auth foundation
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Depends on US1 for user profiles
- **User Story 4 (P3)**: Can start after Foundational (Phase 2) - Depends on US2 for chat history

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tests for a user story marked [P] can run in parallel
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all tests for User Story 1 together (if tests requested):
Task: "Contract test for /api/auth/signup endpoint in auth-server/tests/test_signup.js"
Task: "Contract test for /api/auth/signin endpoint in auth-server/tests/test_signin.js"
Task: "Integration test for user registration journey in auth-server/tests/integration/test_registration.js"

# Launch all models for User Story 1 together:
Task: "Create User model with learning preferences in backend/src/models/user.py"
Task: "Create Session model in backend/src/models/session.py"

# Launch frontend components for User Story 1 together:
Task: "Create signup form component in frontend/src/components/Auth/SignupForm.js"
Task: "Create signin form component in frontend/src/components/Auth/SigninForm.js"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Add User Story 4 → Test independently → Deploy/Demo
6. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (Auth foundation)
   - Developer B: User Story 2 (Chat history)
   - Developer C: User Story 3 (Personalization)
   - Developer D: User Story 4 (Text selection)
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
- Total tasks: 69
- Tasks per story: US1=13, US2=13, US3=10, US4=9, Setup=6, Foundational=8, Polish=9
- Parallel opportunities: 32 tasks marked [P] can run in parallel
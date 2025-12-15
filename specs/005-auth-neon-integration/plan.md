# Implementation Plan: Better Auth + Neon Integration for RAG Chatbot

**Branch**: `005-auth-neon-integration` | **Date**: 2025-12-13 | **Spec**: `/specs/005-auth-neon-integration/spec.md`
**Input**: Feature specification from `/specs/005-auth-neon-integration/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Integrate Better Auth authentication with Neon PostgreSQL persistence to add user accounts, personalized AI responses, and persistent chat history to the existing RAG chatbot. The technical approach involves: 1) Dual-mode authentication (JWT + Better Auth) for backward compatibility, 2) PostgreSQL schema with camelCase naming for Better Auth compatibility, 3) Dynamic system prompt personalization based on user learning preferences, and 4) Gradual migration from in-memory to persistent chat storage.

## Technical Context

**Language/Version**: Python 3.12+ (Backend), Node.js 20+ (Auth Server), TypeScript 5.x (Frontend)
**Primary Dependencies**: FastAPI, Better Auth, Express, React 19, Docusaurus 3.x, OpenAI Agents SDK, Gemini LLM/Embeddings, Qdrant, Neon PostgreSQL
**Storage**: Neon PostgreSQL (User accounts, sessions, chat history), Qdrant (Vector search)
**Testing**: pytest (Backend), Jest/Vitest (Auth Server/Frontend), Postman/curl for API testing
**Target Platform**: Web (Docusaurus static site with React frontend), Linux servers for backend services
**Project Type**: Web application (tri-service architecture: frontend + backend + auth server)
**Performance Goals**: Chat response time within 10% of current baseline (<5s), auth operations <500ms p95, chat history retrieval <200ms p95
**Constraints**: Must maintain backward compatibility with existing JWT authentication, preserve existing RAG functionality, support anonymous users, handle session expiry gracefully
**Scale/Scope**: 1000+ users, 50k+ chat messages, 3 services (frontend, backend, auth server), 5 database tables

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

✅ **I. Service Isolation & Distinct Runtimes**: Maintains tri-service architecture (Frontend Docusaurus/React, Backend FastAPI/Python, Auth Server Node/Express). No direct database access from frontend.

✅ **II. Guardrailed Intelligence**: All chat requests continue through OpenAI Agents SDK guardrail layer. RAG retrieval occurs before LLM generation.

✅ **III. Profile-Driven Personalization**: User background data collected during auth is injected into system prompts for personalized responses.

✅ **IV. Truth in Markdown (Vector Synchronization)**: Qdrant remains derivative of markdown files. Ingest.py script maintains synchronization.

✅ **V. Secure Identity Propagation**: Better Auth is single source of truth for identity. Backend validates sessions via API calls.

✅ **VI. Development Workflow Standards**: Tri-service local development maintained. Environment synchronization via .env files.

## Project Structure

### Documentation (this feature)

```text
specs/005-auth-neon-integration/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command) - COMPLETED
├── data-model.md        # Phase 1 output (/sp.plan command) - COMPLETED
├── contracts/           # Phase 1 output (/sp.plan command) - COMPLETED
│   └── auth-api.yaml    # Better Auth API specification
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Existing tri-service architecture (Option 2: Web application)
backend/
├── src/
│   ├── models/          # SQLAlchemy models (User, Session, ChatHistory)
│   ├── services/        # Chat service, auth validation service
│   ├── api/             # FastAPI endpoints (chat, auth validation)
│   └── database.py      # PostgreSQL connection and session management
├── tests/
│   ├── test_auth.py     # Auth validation tests
│   └── test_chat.py     # Chat history persistence tests
└── requirements.txt

auth-server/
├── src/
│   ├── routes/          # Better Auth routes (signup, signin, signout)
│   ├── middleware/      # CORS, error handling
│   ├── database/        # PostgreSQL client for Better Auth tables
│   └── config/          # Better Auth configuration
├── tests/
│   └── test_auth.js     # Auth endpoint tests
└── package.json

frontend/
├── src/
│   ├── components/
│   │   ├── Auth/        # Login, Signup, Profile components
│   │   └── Chat/        # Enhanced chat with auth context
│   ├── pages/           # Auth pages, profile page
│   ├── services/
│   │   ├── auth.js      # Auth API client
│   │   └── chat.js      # Chat API client with auth headers
│   └── hooks/
│       └── useAuth.js   # Auth state management hook
├── tests/
│   └── test_auth.js     # Frontend auth tests
└── package.json
```

**Structure Decision**: Maintain existing tri-service architecture (Option 2). Backend adds PostgreSQL models and auth validation services. Auth server implements Better Auth endpoints. Frontend adds auth components and services.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

*No constitution violations - all principles maintained.*

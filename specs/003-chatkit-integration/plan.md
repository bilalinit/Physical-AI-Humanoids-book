# Implementation Plan: ChatKit Integration

**Branch**: `003-chatkit-integration` | **Date**: 2025-12-09 | **Spec**: [link to spec.md]
**Input**: Feature specification from `/specs/003-chatkit-integration/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Migrate the existing Docusaurus RAG Chatbot (React/FastAPI) to use OpenAI ChatKit components. This involves replacing the custom ChatBot UI with native ChatKit React components, implementing ChatKit's store for thread management, adapting the existing RAG logic in FastAPI backend to work with ChatKit, and ensuring OpenAI Agents/Guardrails continue to function with proper context management.

## Technical Context

**Language/Version**: TypeScript 5.x (Frontend), Python 3.12+ with FastAPI (Backend), Node.js 20+ (Auth Server)
**Primary Dependencies**: OpenAI ChatKit (Frontend), FastAPI (Backend), Better Auth (Auth Server), Qdrant (Vector DB), Docusaurus 3.x
**Storage**: PostgreSQL (User/Chat History), Qdrant (Vectors), Markdown files (Documentation content)
**Testing**: pytest (Backend), Jest (Frontend), integration tests for ChatKit functionality
**Target Platform**: Web application (Docusaurus site with chat functionality)
**Project Type**: Web application with tri-fold architecture (Frontend/Docusaurus, Backend/FastAPI, Auth Server/Node)
**Performance Goals**: <5 second response time for 95% of queries including vector search and agent processing
**Constraints**: Must maintain existing RAG functionality, preserve OpenAI Agent capabilities, maintain Docusaurus layout compatibility
**Scale/Scope**: Single Docusaurus site with chat functionality for documentation queries

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Post-Design Evaluation:**

1. **Service Isolation & Distinct Runtimes**: ✅ PASSED - ChatKit integration maintains the tri-fold architecture (Frontend/Docusaurus, Backend/FastAPI, Auth Server). ChatKit components run on the frontend but communicate with the existing backend via REST API calls. API contracts ensure proper separation of concerns. CORS configuration for ports 3000, 3001, and 8000 is maintained.

2. **Guardrailed Intelligence**: ✅ PASSED - All chat requests continue to pass through the OpenAI Agents Guardrail layer. The API contract ensures requests route through the existing RAG pipeline with content safety and topic relevance checks. ChatKit components are configured as a frontend interface to the existing backend logic.

3. **Profile-Driven Personalization**: ✅ PASSED - User profile data collected during Auth continues to be injected into System Prompts for RAG queries. The API contract includes user profile information in requests, ensuring personalization is preserved in the ChatKit integration.

4. **Truth in Markdown (Vector Synchronization)**: ✅ PASSED - The Qdrant vector database remains the source of truth derived from Markdown files. ChatKit integration does not affect the ingestion process or vector synchronization. The RAG pipeline remains unchanged, only the frontend interface is updated.

5. **Secure Identity Propagation**: ✅ PASSED - Authentication via Better Auth remains the single source of truth. The Backend continues to validate JWTs independently before processing chat requests through the ChatKit interface. The API contract maintains the same authentication requirements as the original implementation.

## Project Structure

### Documentation (this feature)

```text
specs/003-chatkit-integration/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── main.py              # FastAPI backend with RAG logic
├── src/
│   ├── models/
│   ├── services/
│   └── api/
└── tests/

frontend/
├── src/
│   ├── components/
│   │   └── ChatBot/     # To be replaced with ChatKit components
│   ├── pages/
│   └── services/
└── tests/

auth-server/
├── index.js             # Better Auth implementation
├── src/
│   ├── middleware/
│   └── routes/
└── tests/

specs/
└── 003-chatkit-integration/
    ├── spec.md
    └── plan.md          # This file
```

**Structure Decision**: The existing tri-fold architecture (frontend/backend/auth-server) is maintained. ChatKit integration affects the frontend components in the Docusaurus site and requires backend API adaptations to work with ChatKit's expected interface while preserving existing RAG functionality.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [None identified] | [N/A] | [N/A] |

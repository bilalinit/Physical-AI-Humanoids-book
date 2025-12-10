# Implementation Plan: Text Selection to Chat

**Branch**: `004-text-selection` | **Date**: December 10, 2025 | **Spec**: [specs/004-text-selection/spec.md](/mnt/d/coding Q4/main-hackathon/save-7/hackathon-book/specs/004-text-selection/spec.md)
**Input**: Feature specification from `/specs/004-text-selection/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement text selection functionality that allows users to select text from book content and send it directly to the AI chatbot with proper context integration. The feature includes a custom React hook for text selection detection, a selection popup component, Docusaurus DocItem integration, and backend RAG endpoint enhancements to handle selected text as additional context.

## Technical Context

**Language/Version**: TypeScript 5.x (Frontend), Python 3.12+ (Backend), Node.js 20+ (Auth Server)
**Primary Dependencies**: Docusaurus 3.x, React 19, FastAPI, OpenAI Agents SDK, Qdrant, Better Auth
**Storage**: PostgreSQL (User/Chat History), Qdrant (Vectors), Markdown files (Documentation content)
**Testing**: Jest (Frontend), pytest (Backend), Cypress (E2E)
**Target Platform**: Web application (Docusaurus-based book reader with RAG chatbot integration)
**Project Type**: Web application (frontend + backend + auth server)
**Performance Goals**: <0.5s popup appearance, <3s AI response with text context, 95% success rate for text selection triggering
**Constraints**: <200ms text selection detection, mobile-responsive design, must not interfere with existing chatbot functionality
**Scale/Scope**: Single-page application with Docusaurus integration, works with existing book content structure

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**I. Service Isolation & Distinct Runtimes**: ✅ Compliant - Frontend (Docusaurus/React), Backend (FastAPI/Python), and Auth Server (Node/Express) remain decoupled. Text selection functionality will be implemented in frontend with API calls to backend as needed.

**II. Guardrailed Intelligence**: ✅ Compliant - Selected text will pass through the existing `openai-agents` Guardrail layer, with content safety and topic relevance checks before generation. RAG retrieval (Qdrant) will occur before LLM generation to ground responses in factual data.

**III. Profile-Driven Personalization**: ✅ Compliant - User profile data will be injected into the system prompt along with selected text context for personalized responses.

**IV. Truth in Markdown (Vector Synchronization)**: ✅ Compliant - Selected text from documentation will be used as primary context while still leveraging the Qdrant vector database for additional relevant information.

**V. Secure Identity Propagation**: ✅ Compliant - Authentication will be handled by the existing Better Auth system, with JWT validation in the backend before processing chat requests with selected text.

**VI. Development Workflow Standards**: ✅ Compliant - Implementation will follow the tri-service architecture pattern with separate development for frontend, backend, and auth server.

## Project Structure

### Documentation (this feature)

```text
specs/004-text-selection/
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
├── src/
│   ├── models/
│   ├── services/
│   └── api/
└── tests/

frontend/
├── src/
│   ├── components/
│   │   ├── ChatBot/
│   │   └── TextSelection/
│   ├── theme/
│   │   └── DocItem/
│   └── services/
└── tests/
```

**Structure Decision**: Web application structure selected with frontend changes in frontend/src/components/TextSelection/ and theme/DocItem/, and backend changes to enhance the /api/chat endpoint to handle selected_text parameter.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| N/A | N/A | N/A |

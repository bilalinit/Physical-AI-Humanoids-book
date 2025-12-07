---
id: 1
title: update-docusaurus-constitution
stage: constitution
date: 2025-12-06
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: master
user: user
command: /sp.constitution
labels: [constitution, docusaurus, rag, chatbot]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

```text
# Docusaurus RAG Chatbot Constitution

## Core Principles

### I. Service Isolation & Distinct Runtimes
The architecture is strictly tri-fold: **Frontend (Docusaurus/React)**, **Backend (FastAPI/Python)**, and **Auth Server (Node/Express)**. These must remain decoupled.
*   **Rule:** No direct database access from the Frontend.
*   **Rule:** The Frontend serves static content; dynamic features (Chat, Auth) must occur via REST API calls.
*   **Rule:** Cross-Origin Resource Sharing (CORS) must be explicitly configured to allow communication between ports 3000, 3001, and 8000 (and production equivalents).

### II. Guardrailed Intelligence
AI interactions must never directly expose the raw LLM to the user without mediation.
*   **Rule:** All chat requests must pass through the `openai-agents` Guardrail layer.
*   **Rule:** Every query must be evaluated for **Content Safety** (Pornography/Politics) and **Topic Relevance** (Book-related vs. General) before generation.
*   **Rule:** RAG retrieval (Qdrant) must occur *before* LLM generation to ground the response in factual data.

### III. Profile-Driven Personalization
The chatbot is not generic; it is a tutor that adapts to the user.
*   **Rule:** User background data (Education, Software/Hardware exp) collected during Auth **must** be injected into the System Prompt for every RAG query.
*   **Rule:** If a user is unauthenticated, the system must default to a "General/Beginner" profile but still function for basic queries (if allowed) or prompt for login.

### IV. Truth in Markdown (Vector Synchronization)
The source of truth for knowledge is the `.md` files in the `docs/` directory.
*   **Rule:** The Qdrant vector database is a derivative of the Markdown files.
*   **Rule:** Any content update to documentation requires running the `ingest.py` script to maintain synchronization between the static site and the AI brain.
*   **Rule:** Chunking strategies must preserve context (approx. 1000 chars) to ensure retrieval quality.

### V. Secure Identity Propagation
Authentication and Authorization are handled centrally but verified locally.
*   **Rule:** The **Auth Server** (Better Auth) is the single source of truth for identity.
*   **Rule:** The **Backend** must validate JWTs (from Headers or Cookies) independently before processing chat requests.
*   **Rule:** Secrets (API Keys, DB URLs) must strictly reside in `.env` files and never be committed to version control.

## Technology Standards

### Stack Requirements
*   **Frontend:** Docusaurus 3.x, React 19, TypeScript 5.x.
*   **Backend:** Python 3.12+ (managed via `uv`), FastAPI.
*   **Auth:** Node.js 20+, Better Auth, Express.
*   **Databases:** PostgreSQL (User/Chat History), Qdrant (Vectors).
*   **AI Provider:** Google Gemini (via OpenAI-compatible adapter).

### Directory & Structure
*   `backend/`: Python logic, Ingestion scripts, API.
*   `auth-server/`: Node.js Better Auth implementation.
*   `test-docs/`: Docusaurus site (Frontend).
*   Migrations must be stored in their respective service directories (e.g., `auth-server/migrations/`).

## Development Workflow

### Local Development
1.  **Triple-Terminal Setup:** Developers must run Frontend (:3000), Auth (:3001), and Backend (:8000) simultaneously.
2.  **Ingestion First:** Before testing RAG, the `ingest.py` script must be run against the local `docs/` folder.
3.  **Environment Sync:** `.env` files must be present in all three service roots based on `.env.example`.

### Deployment Gates
1.  **Build Check:** The Docusaurus build (`npm run build`) must pass without broken links (`onBrokenLinks: 'throw'`).
2.  **Linting:** TypeScript types must check out in both Frontend and Auth server.
3.  **Migration Check:** Database schema changes must be applied via SQL scripts before code deployment.

## Governance

### Amendment Process
This Constitution defines the architectural integrity of the RAG Chatbot.
*   Changes to the **Database Schema** (Postgres or Qdrant) require updating Section 8 of the Specifications.
*   Changes to the **API Contract** (between Frontend and Backend) require updating Section 9 of the Specifications.

### Compliance
*   Pull Requests must verify that no "Direct to LLM" calls exist; all must route through the RAG/Guardrail pipeline.
*   Frontend components must treat `apiUrl` as a prop/env variable to ensure compatibility with GitHub Pages (where the backend URL differs from localhost).

**Version**: 1.0.0 | **Ratified**: 2025-12-04 | **Last Amended**: N/A
```

## Response snapshot

Updated the project constitution at .specify/memory/constitution.md with the Docusaurus RAG Chatbot principles. The constitution now includes five core principles: Service Isolation & Distinct Runtimes, Guardrailed Intelligence, Profile-Driven Personalization, Truth in Markdown (Vector Synchronization), and Secure Identity Propagation. Added Technology Standards and Development Workflow sections. Version updated from 1.0.0 to 1.0.1. The updated constitution enforces the tri-fold architecture with Frontend (Docusaurus/React), Backend (FastAPI/Python), and Auth Server (Node/Express) remaining decoupled, with all AI interactions passing through guardrails, user personalization based on profiles, truth in markdown documentation, and secure identity propagation.

## Outcome

- ✅ Impact: Updated constitution file with new Docusaurus RAG Chatbot principles and standards
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md with new content
- 🔁 Next prompts: None needed
- 🧠 Reflection: Constitution now properly defines the architectural principles for the Docusaurus RAG Chatbot project

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
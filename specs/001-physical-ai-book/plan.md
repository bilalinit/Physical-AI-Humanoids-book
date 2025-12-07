# Implementation Plan: Docusaurus Book for Physical AI & Humanoid Robotics

**Branch**: `001-physical-ai-book` | **Date**: 2025-12-06 | **Spec**: specs/001-physical-ai-book/spec.md
**Input**: Feature specification from `/specs/001-physical-ai-book/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Create a comprehensive Docusaurus documentation site that serves as an educational resource for CS/Robotics Engineering Students, bridging modern generative AI (LLMs/VLA) with hard robotics (ROS 2, Hardware Control) using NVIDIA's ecosystem. The site will provide searchable, well-organized content with practical examples, code snippets, and learning paths covering all 12 chapters from hardware setup to capstone projects.

## Technical Context

**Language/Version**: TypeScript 5.x, Python 3.12+ (managed via `uv`)
**Primary Dependencies**: Docusaurus 3.x, React 19, FastAPI, Node.js 20+, Better Auth, Express, Qdrant, PostgreSQL
**Storage**: PostgreSQL (User/Chat History), Qdrant (Vectors), Markdown files (Documentation content)
**Testing**: pytest (Backend), Jest (Auth Server), Docusaurus Jest preset (Frontend)
**Target Platform**: Web-based documentation site (Linux server), with support for multiple browsers and device sizes
**Project Type**: Web application (frontend + backend + auth server)
**Performance Goals**: Site search functionality returns relevant results within 2 seconds for 90% of queries, 95% success rate across different browsers and device sizes
**Constraints**: <200ms p95 for search responses, <10MB memory for documentation serving, offline-capable documentation access
**Scale/Scope**: Targeted for CS/Robotics Engineering Students, 12 chapters with complete, testable code examples, 80% implementation success rate for code examples

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

1. **Service Isolation & Distinct Runtimes**: ✅ COMPLIANT - Architecture is tri-fold with Frontend (Docusaurus/React), Backend (FastAPI/Python), and Auth Server (Node/Express) remaining decoupled. No direct database access from Frontend. Frontend serves static content; dynamic features (Chat, Auth) via REST API calls. CORS configured for ports 3000, 3001, and 8000.

2. **Guardrailed Intelligence**: ✅ COMPLIANT - All AI interactions pass through the `openai-agents` Guardrail layer. Every query is evaluated for Content Safety and Topic Relevance before generation. RAG retrieval (Qdrant) occurs before LLM generation to ground responses in factual data.

3. **Profile-Driven Personalization**: ✅ COMPLIANT - User background data collected during Auth is injected into System Prompt for every RAG query. Unauthenticated users default to "General/Beginner" profile.

4. **Truth in Markdown (Vector Synchronization)**: ✅ COMPLIANT - Source of truth is `.md` files in `docs/` directory. Qdrant vector database is a derivative of Markdown files. Content updates require running `ingest.py` script to maintain synchronization.

5. **Secure Identity Propagation**: ✅ COMPLIANT - Auth Server (Better Auth) is single source of truth for identity. Backend validates JWTs independently before processing chat requests. Secrets in `.env` files never committed to version control.

6. **Development Workflow Standards**: ✅ COMPLIANT - Developers run Frontend (:3000), Auth (:3001), and Backend (:8000) simultaneously. Before testing RAG, `ingest.py` script runs against local `docs/` folder. Environment synchronization requires `.env` files in all three service roots.

## Project Structure

### Documentation (this feature)

```text
specs/001-physical-ai-book/
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
│   ├── pages/
│   └── services/
├── docs/
│   ├── intro.md
│   ├── part-i-infrastructure/
│   │   ├── index.md
│   │   ├── chapter-1-hardware-os-config.md
│   │   └── chapter-2-edge-ecosystem.md
│   ├── part-ii-ros-nervous-system/
│   │   ├── index.md
│   │   ├── chapter-3-ros2-architecture.md
│   │   └── chapter-4-urdf-kinematics.md
│   ├── part-iii-digital-twin/
│   │   ├── index.md
│   │   ├── chapter-5-physics-gazebo.md
│   │   └── chapter-6-isaac-sim.md
│   ├── part-iv-perception-navigation/
│   │   ├── index.md
│   │   ├── chapter-7-sensors-vslam.md
│   │   └── chapter-8-navigation.md
│   ├── part-v-vla-integration/
│   │   ├── index.md
│   │   ├── chapter-9-voice-pipeline.md
│   │   ├── chapter-10-brain-llm.md
│   │   └── chapter-11-vla.md
│   └── part-vi-capstone/
│       ├── index.md
│       └── chapter-12-autonomous-humanoid.md
├── static/
│   └── img/
├── src/
│   ├── components/
│   └── pages/
├── docusaurus.config.ts
├── sidebars.ts
└── package.json

auth-server/
├── src/
│   ├── models/
│   ├── services/
│   └── middleware/
└── tests/

# Documentation content for RAG
docs/
├── intro.md
├── part-i-infrastructure/
├── part-ii-ros-nervous-system/
├── part-iii-digital-twin/
├── part-iv-perception-navigation/
├── part-v-vla-integration/
└── part-vi-capstone/
```

**Structure Decision**: Web application structure selected with three distinct services (frontend, backend, auth-server) to maintain proper separation of concerns as per constitution. The frontend will host the Docusaurus-based documentation site with 12 chapters organized across 6 parts as specified in the feature requirements.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|

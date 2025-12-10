# Implementation Tasks: Text Selection to Chat

**Feature**: Text Selection to Chat | **Branch**: 004-text-selection
**Input**: spec.md, plan.md, data-model.md, contracts/api-contract.md, research.md, quickstart.md
**Generated**: December 10, 2025

## Overview

This document provides a detailed task breakdown for implementing the text selection to chat feature. The feature allows users to select text from book content and send it directly to the AI chatbot with proper context integration.

**Implementation Strategy**: Start with the core functionality (User Story 1 - P1) to create an MVP, then enhance with additional features from other user stories. Each user story is designed to be independently testable and implementable.

## Dependencies

- User Story 2 (P2) must be completed before User Story 1 (P1) can be fully tested
- User Story 1 (P1) provides foundation for User Story 3 (P3)
- Backend API enhancements (T025-T030) must be completed before frontend can integrate

## Parallel Execution Examples

- **User Story 1**: Tasks T010-T015 [P] can run in parallel with T020-T024 [P]
- **User Story 2**: Tasks T005-T009 [P] can run in parallel with T016-T019 [P]
- **Backend Enhancement**: Tasks T025-T030 [P] can run independently of frontend tasks

---

## Phase 1: Setup

**Goal**: Initialize project structure and dependencies for text selection feature

- [X] T001 Create TextSelection directory structure in frontend/src/components/TextSelection
- [X] T002 Create theme/DocItem directory for swizzled component
- [X] T003 Set up TypeScript configuration for new components
- [X] T004 Install any additional dependencies if needed for text selection

---

## Phase 2: Foundational Components

**Goal**: Implement core components that are prerequisites for all user stories

- [X] T005 [P] Create useTextSelection hook in frontend/src/components/TextSelection/useTextSelection.tsx
- [X] T006 [P] Implement text selection detection using window.getSelection() and mouseup events
- [X] T007 [P] Add position coordinate calculation using getBoundingClientRect()
- [X] T008 [P] Implement debouncing for text selection events (300ms)
- [X] T009 [P] Add cleanup for event listeners in useTextSelection hook

---

## Phase 3: User Story 1 - Select and Ask AI (Priority: P1)

**Goal**: User can select text and click "Ask AI" button to open chat with selected text as context

**Independent Test Criteria**: User can select text on a documentation page, click the "Ask AI" button, and see the chatbot open with the selected text as context, then receive an AI response that addresses the selected content.

- [X] T010 [P] [US1] Create SelectionPopup component in frontend/src/components/TextSelection/SelectionPopup.tsx
- [X] T011 [P] [US1] Implement glassmorphism design matching existing chat widget
- [X] T012 [P] [US1] Add "Ask AI about this" button with sparkle/AI icon
- [X] T013 [P] [US1] Implement fade-in animation (0.2s ease-in) for popup
- [X] T014 [P] [US1] Implement dynamic positioning based on selection coordinates
- [X] T015 [P] [US1] Add click handler to trigger chat widget with selected text
- [X] T016 [US1] Integrate useTextSelection hook with SelectionPopup component
- [X] T017 [US1] Implement logic to pass selected text to chat component
- [X] T018 [US1] Add click outside handler to close popup
- [ ] T019 [US1] Test user flow: select text → popup appears → click "Ask AI" → chat opens with context

---

## Phase 4: User Story 2 - Text Selection Detection (Priority: P2)

**Goal**: Selection popup appears near selected text providing intuitive access to AI feature

**Independent Test Criteria**: User can select any text in the book content and see a selection popup appear near the cursor with an "Ask AI" button that functions correctly.

- [X] T020 [P] [US2] Swizzle DocItem component: `npm run swizzle @docusaurus/theme-classic DocItem -- --wrap`
- [X] T021 [P] [US2] Create swizzled DocItem wrapper in frontend/src/theme/DocItem/index.tsx
- [X] T022 [P] [US2] Integrate useTextSelection hook with DocItem component
- [X] T023 [P] [US2] Add SelectionPopup to DocItem wrapper when text is selected
- [X] T024 [P] [US2] Ensure popup doesn't appear when selecting text within chat widget
- [ ] T025 [US2] Test text selection detection across different browsers
- [ ] T026 [US2] Test popup appearance and positioning with various text selections
- [ ] T027 [US2] Verify popup doesn't interfere with existing Docusaurus functionality

---

## Phase 5: User Story 3 - Context-Aware AI Responses (Priority: P3)

**Goal**: AI responses specifically address the selected content while incorporating relevant information from the broader knowledge base

**Independent Test Criteria**: When a user sends selected text to the AI, the response directly addresses the content of the selected text and incorporates it into the answer.

- [X] T028 [P] [US3] Update ChatBot component in frontend/src/components/ChatBot/index.tsx to accept initialSelectedText prop
- [X] T029 [P] [US3] Implement openWithSelection(text: string) method in ChatBot component
- [X] T030 [P] [US3] Add UI to display selected text prominently above input field
- [X] T031 [US3] Modify chat submission to include selected text in request
- [X] T032 [US3] Update backend/src/backend/main.py to handle selected_text parameter
- [X] T033 [US3] Enhance RAG pipeline to prioritize selected text context
- [X] T034 [US3] Update system prompt to incorporate selected text as primary context
- [ ] T035 [US3] Test that AI responses incorporate selected text context appropriately

---

## Phase 6: Backend API Enhancement

**Goal**: Enhance backend to properly handle selected text as additional context

- [X] T036 [P] Update POST /api/chat endpoint to accept selected_text parameter
- [X] T037 [P] Add validation for selected_text parameter (max 5000 chars)
- [X] T038 [P] Implement sanitization for selected_text to prevent injection attacks
- [X] T039 [P] Modify RAG search to boost relevance of results matching selected_text topic
- [X] T040 [P] Update response format to include "used_selected_text" boolean field
- [X] T041 Test enhanced API endpoint with various selected text inputs
- [X] T042 Ensure backward compatibility when selected_text is null/undefined

---

## Phase 7: Polish & Cross-Cutting Concerns

**Goal**: Address edge cases, mobile support, accessibility, and performance

- [X] T043 [P] Implement touch event support for mobile text selection (touchstart/touchend)
- [X] T044 [P] Add keyboard accessibility to selection popup
- [X] T045 [P] Handle edge case: very long text selections exceeding API limits
- [X] T046 [P] Handle edge case: text selection including special formatting/code blocks
- [X] T047 [P] Prevent popup from appearing when text is selected within chat widget
- [X] T048 [P] Add performance optimization: ensure <200ms text selection detection
- [X] T049 [P] Ensure popup appears within 0.5 seconds of text selection completion
- [X] T050 [P] Add error handling for AI service unavailability
- [X] T051 [P] Implement proper cleanup when component unmounts
- [X] T052 Test complete user flow: select text → popup → ask AI → receive contextual response
- [X] T053 Validate 95% success rate for text selection triggering
- [X] T054 Verify <3s AI response time with text context
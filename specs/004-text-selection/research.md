# Research: Text Selection to Chat Feature

## Overview
This document captures research findings for implementing the text selection to chat feature, which allows users to select text from book content and send it directly to the AI chatbot with proper context integration.

## Decision: Text Selection Detection Approach
**Rationale**: The feature requires detecting when a user selects text on the page and capturing the selected text along with its position. The standard browser API approach using `window.getSelection()` and mouseup events is the most reliable cross-browser solution.

**Alternatives considered**:
- MutationObserver approach: More complex and not suitable for text selection detection
- Custom contentEditable wrapper: Would interfere with Docusaurus content rendering
- Range-based selection with mousedown/mouseup: Standard approach that works reliably

## Decision: Selection Popup Component Design
**Rationale**: A floating popup button near the selected text provides an intuitive user experience for triggering the AI chat. The glassmorphism design matches the existing chat widget aesthetic while providing clear visual feedback.

**Alternatives considered**:
- Context menu approach: Would require right-click which isn't standard on mobile
- Toolbar at top of page: Would be distant from the selected text
- Highlight with inline button: More complex to implement with positioning challenges

## Decision: Docusaurus Theme Swizzling Strategy
**Rationale**: Swizzling the DocItem component allows integration of text selection detection without modifying Docusaurus core components. This preserves upgrade paths while enabling the required functionality.

**Alternatives considered**:
- Global event listeners: Would be harder to manage scope and context
- Custom MDX components: Would require changing all existing documentation pages
- Layout wrapper approach: Less targeted than DocItem swizzling

## Decision: Backend API Enhancement Strategy
**Rationale**: Extending the existing `/api/chat` endpoint with a `selected_text` parameter maintains API consistency while providing the additional context needed for the feature. This approach leverages the existing RAG pipeline.

**Alternatives considered**:
- New dedicated endpoint: Would duplicate functionality and increase complexity
- Query parameters: Less structured than request body
- WebSocket approach: Unnecessary complexity for this use case

## Decision: State Management for Selected Text
**Rationale**: Using React state to manage selected text context in the ChatBot component allows for proper UI updates and context preservation during the chat session.

**Alternatives considered**:
- Global context: Unnecessary for this feature scope
- URL parameters: Would expose sensitive text content
- Local storage: Would persist between sessions inappropriately

## Key Technical Considerations

1. **Mobile Support**: Touch events (touchstart/touchend) will need to be implemented alongside mouse events for mobile devices.

2. **Performance**: Debouncing text selection events will prevent excessive re-renders during text selection operations.

3. **Accessibility**: The selection popup should be keyboard accessible and respect user preferences.

4. **Security**: Selected text should be sanitized before being sent to the backend to prevent injection attacks.

5. **Privacy**: Text selection data should not be stored or logged unnecessarily.

## Implementation Dependencies

- `window.getSelection()` API for text selection detection
- `getBoundingClientRect()` for positioning the selection popup
- React hooks for state management
- Existing backend API structure for chat functionality
- Docusaurus theme customization capabilities
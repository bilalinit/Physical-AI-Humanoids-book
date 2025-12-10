# Data Model: Text Selection to Chat

## Overview
This document defines the data structures and entities for the text selection to chat feature.

## Key Entities

### SelectedText
The text content that user has selected on the page, including position coordinates for popup placement.

**Fields**:
- `text` (string): The actual selected text content
- `position` (object): Coordinates for popup placement
  - `x` (number): X coordinate for popup positioning
  - `y` (number): Y coordinate for popup positioning
- `rect` (object): Bounding rectangle of the selection
  - `top` (number): Top coordinate of selection
  - `left` (number): Left coordinate of selection
  - `width` (number): Width of selection
  - `height` (number): Height of selection
- `timestamp` (number): Time when selection was made
- `hasSelection` (boolean): Whether text is currently selected

**Validation**:
- Text must not be empty or whitespace-only
- Position coordinates must be positive numbers
- Text length should be reasonable (not exceed API limits)

### SelectionPopup
UI element that appears near selected text with button to trigger AI interaction.

**Fields**:
- `isVisible` (boolean): Whether the popup is currently visible
- `position` (object): Position coordinates for the popup
  - `x` (number): X coordinate for popup positioning
  - `y` (number): Y coordinate for popup positioning
- `selectedText` (string): Reference to the selected text
- `onAskAI` (function): Callback function to trigger AI interaction
- `onClose` (function): Callback function to close the popup

**Validation**:
- Position coordinates must be valid screen coordinates
- Must have either selectedText or a valid selection reference

### ChatContext
Enhanced context data structure that includes both selected text and existing RAG context.

**Fields**:
- `selectedText` (string | null): The text selected by the user (optional)
- `userQuery` (string): The user's query to the AI
- `chatHistory` (array): Previous messages in the conversation
- `userId` (string | null): ID of the authenticated user (optional)
- `userProfile` (object | null): User profile data for personalization (optional)
- `ragContext` (object): Existing RAG-retrieved context from vector database

**Validation**:
- If selectedText is provided, it must be properly sanitized
- userQuery must not be empty
- chatHistory must be properly formatted array of messages

## State Transitions

### Text Selection State
1. `IDLE` - No text is selected
2. `SELECTING` - User is in the process of selecting text
3. `SELECTED` - Text has been selected, popup is visible
4. `ASKING_AI` - User has clicked "Ask AI", chat is opening with context
5. `DESELECTED` - User has deselected text or closed popup

### Popup Visibility State
1. `HIDDEN` - Popup is not visible
2. `APPEARING` - Popup is animating in
3. `VISIBLE` - Popup is fully visible
4. `DISAPPEARING` - Popup is animating out
5. `CLOSED` - Popup has been closed

## API Data Flow

### Frontend to Backend
```
User selects text → SelectedText object → ChatContext object → API request
```

### Request Structure
```json
{
  "user_query": "string",
  "selected_text": "string | null",
  "chat_history": "array",
  "user_profile": "object | null"
}
```

### Response Structure
```json
{
  "output": "string",
  "context_chunks": "array",
  "sources": "array",
  "used_selected_text": "boolean"
}
```

## Relationships

- `SelectedText` is contained within `ChatContext` when text selection is active
- `SelectionPopup` references `SelectedText` for display purposes
- `ChatContext` extends the existing chat context with selected text information
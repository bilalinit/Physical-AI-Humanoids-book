# Quickstart: Text Selection to Chat Feature

## Overview
This guide provides a quick setup and usage guide for the text selection to chat feature that allows users to select text from book content and send it directly to the AI chatbot.

## Prerequisites
- Docusaurus-based book reader with RAG chatbot integration
- Running backend service with enhanced chat endpoint
- Proper CORS configuration between frontend (port 3000) and backend (port 8000)

## Implementation Steps

### 1. Frontend Components
1. Create the text selection hook:
   - File: `frontend/src/components/TextSelection/useTextSelection.tsx`
   - Implements text selection detection using browser APIs
   - Returns selected text and position coordinates

2. Create the selection popup component:
   - File: `frontend/src/components/TextSelection/SelectionPopup.tsx`
   - Displays "Ask AI about this" button near selected text
   - Implements glassmorphism design matching chat widget

3. Integrate with Docusaurus:
   - Swizzle DocItem component: `npm run swizzle @docusaurus/theme-classic DocItem -- --wrap`
   - Create: `frontend/src/theme/DocItem/index.tsx`
   - Wrap content with text selection detection

4. Update ChatBot component:
   - Modify: `frontend/src/components/ChatBot/index.tsx`
   - Add support for initialSelectedText prop
   - Implement openWithSelection method

### 2. Backend Enhancement
1. Update chat endpoint in `backend/src/backend/main.py`
2. Add selected_text parameter handling to POST /api/chat
3. Enhance RAG pipeline to prioritize selected text context
4. Update system prompt to incorporate selected text

### 3. Testing
1. Verify text selection detection works across different browsers
2. Test popup appearance and positioning
3. Validate chat functionality with selected text context
4. Ensure mobile touch events work properly

## Usage Flow
1. User selects text on any book content page
2. Selection popup appears near the selected text
3. User clicks "Ask AI about this" button
4. Chat panel opens with selected text as context
5. AI response incorporates selected text as primary context

## Key Configuration Points
- Debounce time for text selection events: 300ms
- Minimum text selection length: 5 characters
- Popup positioning offset: 10px above selection
- Maximum selected text length: 5000 characters (for API limits)

## Troubleshooting
- If popup doesn't appear, check that text selection events are being detected
- If chat doesn't open with context, verify API endpoint is receiving selected_text parameter
- For positioning issues, ensure getBoundingClientRect() is working with your layout
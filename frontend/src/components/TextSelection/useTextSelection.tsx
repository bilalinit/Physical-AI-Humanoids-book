import { useState, useEffect, useCallback } from 'react';

interface SelectedText {
  text: string;
  position: {
    x: number;
    y: number;
  };
  rect: {
    top: number;
    left: number;
    width: number;
    height: number;
  };
  timestamp: number;
  hasSelection: boolean;
}

export const useTextSelection = () => {
  const [selectedText, setSelectedText] = useState<SelectedText | null>(null);
  const [debounceTimer, setDebounceTimer] = useState<NodeJS.Timeout | null>(null);

  const getSelectedText = useCallback((): SelectedText | null => {
    const selection = window.getSelection();

    if (!selection || selection.toString().trim() === '') {
      return null;
    }

    // Check if the selection is within the chat widget
    const anchorNode = selection.anchorNode;
    let currentNode = anchorNode?.parentElement;
    while (currentNode) {
      if (currentNode.classList &&
          (currentNode.classList.contains('chatkit') ||
           currentNode.classList.contains('chatbot') ||
           currentNode.classList.contains('chat-container') ||
           currentNode.tagName.toLowerCase() === 'iframe')) {
        return null; // Don't show popup if selection is within chat widget
      }
      currentNode = currentNode.parentElement;
    }

    let text = selection.toString().trim();

    // Clean up the selected text by removing extra whitespace and normalizing line breaks
    text = text.replace(/\s+/g, ' '); // Replace multiple whitespace with single space
    text = text.trim();

    // Don't process if text is too short
    if (text.length < 5) {
      return null;
    }

    // Limit text length to prevent API limits (max 5000 chars as per backend validation)
    if (text.length > 5000) {
      return null; // Don't show popup for very long selections
    }

    // Get the bounding rectangle of the selection
    let range;
    try {
      range = selection.getRangeAt(0);
    } catch (e) {
      // If no range is available, return null
      return null;
    }

    const rect = range.getBoundingClientRect();

    // Calculate position - place popup above the selection
    const position = {
      x: rect.left + window.scrollX,
      y: rect.top + window.scrollY - 40, // 40px above selection
    };

    return {
      text,
      position,
      rect: {
        top: rect.top,
        left: rect.left,
        width: rect.width,
        height: rect.height,
      },
      timestamp: Date.now(),
      hasSelection: true,
    };
  }, []);

  const handleSelection = useCallback(() => {
    // Clear any existing debounce timer
    if (debounceTimer) {
      clearTimeout(debounceTimer);
    }

    // Set a new debounce timer (300ms)
    const timer = setTimeout(() => {
      const currentSelection = getSelectedText();
      setSelectedText(currentSelection);
    }, 300);

    setDebounceTimer(timer);
  }, [debounceTimer, getSelectedText]);

  useEffect(() => {
    // Add event listeners for text selection
    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('touchend', handleSelection);
    // Also add touchmove to handle selection changes during touch
    document.addEventListener('touchmove', handleSelection);

    // Clean up event listeners
    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('touchend', handleSelection);
      document.removeEventListener('touchmove', handleSelection);

      // Clear any pending timers
      if (debounceTimer) {
        clearTimeout(debounceTimer);
      }
    };
  }, [handleSelection, debounceTimer]);

  // Function to clear selection
  const clearSelection = useCallback(() => {
    setSelectedText(null);
    window.getSelection()?.empty();
  }, []);

  return {
    selectedText,
    clearSelection,
  };
};
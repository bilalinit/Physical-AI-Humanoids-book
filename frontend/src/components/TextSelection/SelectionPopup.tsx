import React, { useEffect, useRef } from 'react';
import './SelectionPopup.css';

interface SelectionPopupProps {
  isVisible: boolean;
  position: {
    x: number;
    y: number;
  };
  selectedText: string;
  onAskAI: (text: string) => void;
  onClose: () => void;
}

const SelectionPopup: React.FC<SelectionPopupProps> = ({
  isVisible,
  position,
  selectedText,
  onAskAI,
  onClose,
}) => {
  const popupRef = useRef<HTMLDivElement>(null);

  // Handle click outside to close popup
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (popupRef.current && !popupRef.current.contains(event.target as Node)) {
        onClose();
      }
    };

    if (isVisible) {
      document.addEventListener('mousedown', handleClickOutside);
    }

    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [isVisible, onClose]);

  // Don't render if not visible
  if (!isVisible) {
    return null;
  }

  // Ensure the popup stays within the viewport
  const adjustedPosition = {
    left: Math.max(10, Math.min(position.x, window.innerWidth - 200)),
    top: Math.max(10, position.y),
  };

  return (
    <div
      ref={popupRef}
      className="selection-popup"
      style={{
        left: `${adjustedPosition.left}px`,
        top: `${adjustedPosition.top}px`,
      }}
    >
      <button
        className="ask-ai-button"
        onClick={() => onAskAI(selectedText)}
        onKeyDown={(e) => {
          if (e.key === 'Enter' || e.key === ' ') {
            e.preventDefault();
            onAskAI(selectedText);
          }
        }}
        aria-label="Ask AI about selected text"
        tabIndex={0}
      >
        <span className="ai-icon">✨</span>
        <span>Ask AI</span>
      </button>
    </div>
  );
};

export default SelectionPopup;
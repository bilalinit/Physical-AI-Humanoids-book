import React, { useEffect } from 'react';
import { useTextSelection } from '@site/src/components/TextSelection/useTextSelection';
import SelectionPopup from '@site/src/components/TextSelection/SelectionPopup';
import OriginalDocItem from '@theme-original/DocItem';
import type { Props } from '@theme/DocItem';

const DocItem: React.FC<Props> = (props) => {
  const { selectedText, clearSelection } = useTextSelection();

  // Function to handle asking AI with selected text
  const handleAskAI = (text: string) => {
    // Format the message with "Explain this:" prompt
    const message = `Explain this: ${text}`;

    // Trigger an event that the ChatBot component can listen to
    window.dispatchEvent(new CustomEvent('askAIWithSelection', { detail: { message, text } }));
    clearSelection();
  };

  return (
    <>
      <OriginalDocItem {...props} />
      {selectedText && (
        <SelectionPopup
          isVisible={!!selectedText}
          position={selectedText.position}
          selectedText={selectedText.text}
          onAskAI={handleAskAI}
          onClose={clearSelection}
        />
      )}
    </>
  );
};

export default DocItem;
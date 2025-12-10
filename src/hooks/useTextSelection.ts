import { useState, useEffect } from 'react';

export interface TextSelection {
  text: string;
  isActive: boolean;
}

export function useTextSelection() {
  const [selection, setSelection] = useState<TextSelection>({
    text: '',
    isActive: false
  });

  useEffect(() => {
    const handleSelection = () => {
      const selectedText = window.getSelection()?.toString().trim();

      if (selectedText && selectedText.length > 10) {
        // Truncate if too long (2000 token limit ~= 8000 chars)
        const truncated = selectedText.slice(0, 8000);
        setSelection({
          text: truncated,
          isActive: true
        });
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, []);

  const clearSelection = () => {
    setSelection({ text: '', isActive: false });
  };

  return { selection, clearSelection };
}
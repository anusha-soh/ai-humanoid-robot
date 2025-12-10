import { useState, useCallback } from 'react';
import { chatClient, ChatMessage } from '../api/chatClient';

export function useChat() {
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [isStreaming, setIsStreaming] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const sendMessage = useCallback(async (
    userMessage: string,
    mode: 'general' | 'selection',
    context?: string
  ) => {
    setIsStreaming(true);
    setError(null);

    // Add user message
    const userMsg: ChatMessage = { role: 'user', content: userMessage };
    setMessages(prev => [...prev, userMsg]);

    // Stream assistant response
    let assistantContent = '';
    let sources = [];

    try {
      for await (const chunk of chatClient.streamChat({
        message: userMessage,
        mode,
        context
      })) {
        if (chunk.content) {
          assistantContent += chunk.content;
          setMessages(prev => {
            const updated = [...prev];
            const lastMsg = updated[updated.length - 1];

            if (lastMsg?.role === 'assistant') {
              updated[updated.length - 1] = {
                ...lastMsg,
                content: assistantContent
              };
            } else {
              updated.push({ role: 'assistant', content: assistantContent });
            }
            return updated;
          });
        }

        if (chunk.sources) {
          sources = chunk.sources;
        }
      }

      // Add sources to final message
      if (sources.length > 0) {
        setMessages(prev => {
          const updated = [...prev];
          updated[updated.length - 1].sources = sources;
          return updated;
        });
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Unknown error');
    } finally {
      setIsStreaming(false);
    }
  }, []);

  const clearMessages = useCallback(() => {
    setMessages([]);
    setError(null);
  }, []);

  return { messages, isStreaming, error, sendMessage, clearMessages };
}
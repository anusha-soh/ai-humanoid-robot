import React, { useState, useRef, useEffect } from 'react';
import { useChat } from '../../hooks/useChat';
import { useTextSelection } from '../../hooks/useTextSelection';
import styles from './ChatWidget.module.css';

export default function ChatWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const [input, setInput] = useState('');
  const { messages, isStreaming, error, sendMessage } = useChat();
  const { selection, clearSelection } = useTextSelection();
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!input.trim() || isStreaming) return;

    const mode = selection.isActive ? 'selection' : 'general';
    const context = selection.isActive ? selection.text : undefined;

    await sendMessage(input, mode, context);
    setInput('');
  };

  return (
    <>
      {/* Floating Button */}
      <button
        className={styles.floatingButton}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Toggle chat"
      >
        💬
      </button>

      {/* Chat Panel */}
      {isOpen && (
        <div className={styles.chatPanel}>
          <div className={styles.chatHeader}>
            <h3>AI Assistant</h3>
            {selection.isActive && (
              <div className={styles.selectionBadge}>
                Selection Mode Active
                <button onClick={clearSelection}>✕</button>
              </div>
            )}
          </div>

          <div className={styles.messageList}>
            {messages.length === 0 && (
              <div className={styles.welcomeMessage}>
                👋 Ask me anything about the book!
              </div>
            )}

            {messages.map((msg, idx) => (
              <div
                key={idx}
                className={`${styles.message} ${styles[msg.role as keyof typeof styles]}`}
              >
                <div className={styles.messageContent}>
                  {msg.content}
                </div>

                {msg.sources && msg.sources.length > 0 && (
                  <div className={styles.sources}>
                    <strong>Sources:</strong>
                    {msg.sources.map((src, i) => (
                      <a
                        key={i}
                        href={`/${src.file.replace('.md', '')}`}
                        className={styles.sourceLink}
                      >
                        [{i + 1}] {src.heading}
                      </a>
                    ))}
                  </div>
                )}
              </div>
            ))}

            {error && (
              <div className={styles.error}>Error: {error}</div>
            )}

            <div ref={messagesEndRef} />
          </div>

          <form onSubmit={handleSubmit} className={styles.inputForm}>
            <input
              type="text"
              value={input}
              onChange={(e) => setInput(e.target.value)}
              placeholder="Ask a question..."
              disabled={isStreaming}
              className={styles.input}
            />
            <button
              type="submit"
              disabled={isStreaming || !input.trim()}
              className={styles.submitButton}
            >
              {isStreaming ? '⏳' : '➤'}
            </button>
          </form>
        </div>
      )}
    </>
  );
}
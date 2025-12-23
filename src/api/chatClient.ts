export interface ChatMessage {
  role: 'user' | 'assistant';
  content: string;
  sources?: Source[];
  conversation_id?: string;
}

export interface Source {
  file: string;
  heading: string;
  similarity: number;
}

export interface ChatRequestBody {
  message: string;
  mode: 'general' | 'selection';
  context?: string;
  conversation_id?: string;
}

export class ChatClient {
  private baseUrl: string;

  constructor() {
    // Use REACT_APP_API_BASE_URL from environment, fallback to localhost for development
    this.baseUrl = process.env.REACT_APP_API_BASE_URL ||
                  (process.env.NODE_ENV === 'production'
                    ? 'https://anusha-soh-rag-chatbot.hf.space'  // HF Spaces backend
                    : 'http://localhost:8000');
  }

  async *streamChat(request: ChatRequestBody): AsyncGenerator<ChatMessage, void, unknown> {
    const response = await fetch(`${this.baseUrl}/api/chat`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(request),
    });

    if (!response.ok) {
      throw new Error(`Chat API error: ${response.statusText}`);
    }

    const reader = response.body?.getReader();
    const decoder = new TextDecoder();
    let buffer = '';

    while (true) {
      const { done, value } = await reader!.read();
      if (done) break;

      buffer += decoder.decode(value, { stream: true });
      const lines = buffer.split('\n');
      buffer = lines.pop() || '';

      for (const line of lines) {
        if (line.startsWith('data: ')) {
          const data = JSON.parse(line.slice(6));

          if (data.type === 'metadata') {
            yield {
              role: 'assistant',
              content: '',
              conversation_id: data.conversation_id
            };
          } else if (data.type === 'token') {
            yield { role: 'assistant', content: data.content };
          } else if (data.type === 'done') {
            yield { role: 'assistant', content: '', sources: data.sources };
          }
        }
      }
    }
  }

  async healthCheck() {
    const response = await fetch(`${this.baseUrl}/api/health`);
    return response.json();
  }
}

export const chatClient = new ChatClient();
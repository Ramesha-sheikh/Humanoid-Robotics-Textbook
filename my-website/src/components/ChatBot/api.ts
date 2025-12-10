/**
 * API Client for RAG Chatbot Backend
 * Connects to FastAPI backend with streaming support
 */

const API_BASE_URL = process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8000';

export interface ChatRequest {
  query: string;
  mode: 'normal' | 'highlight';
  highlight_context?: {
    text: string;
    startPage: number;
    endPage: number;
    chapterSlug: string;
  };
}

export interface Source {
  chapter: string;
  section: string;
  page: number;
}

export interface ChatResponse {
  answer: string;
  sources: Source[];
  latency_ms: number;
  mode: string;
}

export interface StreamChunk {
  token: string;
  done: boolean;
  error?: string;
}

/**
 * Send chat request and get complete response (non-streaming)
 */
export async function sendChatMessage(request: ChatRequest): Promise<ChatResponse> {
  const response = await fetch(`${API_BASE_URL}/chat`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(request),
  });

  if (!response.ok) {
    throw new Error(`Chat API error: ${response.statusText}`);
  }

  return response.json();
}

/**
 * Send chat request and stream response tokens via SSE
 */
export async function* streamChatMessage(request: ChatRequest): AsyncGenerator<StreamChunk> {
  const response = await fetch(`${API_BASE_URL}/chat/stream`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(request),
  });

  if (!response.ok) {
    throw new Error(`Stream API error: ${response.statusText}`);
  }

  const reader = response.body?.getReader();
  const decoder = new TextDecoder();

  if (!reader) {
    throw new Error('Response body is not readable');
  }

  let buffer = '';

  while (true) {
    const { done, value } = await reader.read();

    if (done) break;

    // Decode chunk and add to buffer
    buffer += decoder.decode(value, { stream: true });

    // Process complete SSE messages (data: {...}\n\n)
    const lines = buffer.split('\n\n');
    buffer = lines.pop() || ''; // Keep incomplete message in buffer

    for (const line of lines) {
      if (line.startsWith('data: ')) {
        const data = line.slice(6); // Remove "data: " prefix

        try {
          const chunk: StreamChunk = JSON.parse(data);
          yield chunk;

          if (chunk.done || chunk.error) {
            return;
          }
        } catch (e) {
          console.error('Failed to parse SSE message:', data, e);
        }
      }
    }
  }
}

/**
 * Check backend health
 */
export async function checkHealth(): Promise<{ status: string; version: string }> {
  const response = await fetch(`${API_BASE_URL}/health`);

  if (!response.ok) {
    throw new Error('Backend health check failed');
  }

  return response.json();
}

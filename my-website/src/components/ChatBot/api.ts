/**
 * API Client for RAG Chatbot Backend
 * Connects to FastAPI backend with streaming support
 */

const API_BASE_URL = 'http://localhost:8001';

export interface ChatRequest {
  question: string;
  selected_text?: string;
}

export interface Source {
  chapter?: string;
  section?: string;
  page?: string;
  url?: string;
}

export interface ChatResponse {
  answer: string;
  sources: Source[];
}


export interface StreamChunk {
  token?: string;
  done: boolean;
  error?: string;
  response?: ChatResponse;
}

/**
 * Send chat request and get complete response (non-streaming)
 */
export async function sendChatMessage(question: string, selected_text?: string): Promise<ChatResponse> {
  const requestBody = { question, selected_text };
  const response = await fetch(`${API_BASE_URL}/chat`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(requestBody),
  });

  if (!response.ok) {
    throw new Error(`Chat API error: ${response.statusText}`);
  }

  return response.json();
}

/**
 * Send chat request and stream response tokens via SSE
 */
export async function* streamChatMessage(question: string, selected_text?: string): AsyncGenerator<StreamChunk> {
  const requestBody = { question, selected_text };
  const response = await fetch(`${API_BASE_URL}/stream-chat`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(requestBody),
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
          const parsedData = JSON.parse(data);

          // Handle both token chunks and final response chunks
          if (parsedData.done && parsedData.response) {
            // This is the final chunk with full response
            const finalChunk: StreamChunk = {
              done: true,
              response: JSON.parse(parsedData.response) // Parse the response from string to object
            };
            yield finalChunk;
            return;
          } else {
            // This is a regular token chunk
            const chunk: StreamChunk = parsedData;
            yield chunk;

            if (chunk.done || chunk.error) {
              return;
            }
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

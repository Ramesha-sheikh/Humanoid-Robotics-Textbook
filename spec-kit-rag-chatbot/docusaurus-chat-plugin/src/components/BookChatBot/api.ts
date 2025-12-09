interface ChatResponseSource {
  title: string;
  url: string;
  heading?: string;
}

export interface ChatMessageData {
  type: 'text' | 'final';
  content?: string; // For 'text' type
  answer?: string; // For 'final' type
  sources?: ChatResponseSource[]; // For 'final' type
  session_id?: string;
}

interface ChatRequest {
  user_query: string;
  session_id?: string;
  user_id?: string;
}

const BACKEND_URL = process.env.BACKEND_API_URL || 'http://localhost:8000'; // Default to localhost

export async function streamChatResponse(
  query: string,
  onMessage: (data: ChatMessageData) => void,
  onError: (error: Event) => void,
  sessionId?: string,
  userId?: string
): Promise<void> {
  const requestBody: ChatRequest = {
    user_query: query,
    session_id: sessionId,
    user_id: userId,
  };

  try {
    const response = await fetch(`${BACKEND_URL}/api/chat`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(requestBody),
    });

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    const reader = response.body?.getReader();
    if (!reader) {
      throw new Error('Failed to get reader for response body.');
    }

    const decoder = new TextDecoder('utf-8');
    let buffer = '';

    while (true) {
      const { value, done } = await reader.read();
      if (done) {
        break;
      }
      buffer += decoder.decode(value, { stream: true });

      // Process complete lines
      const lines = buffer.split('\\n\\n');
      buffer = lines.pop() || ''; // Keep incomplete last line in buffer

      for (const line of lines) {
        if (line.startsWith('data: ')) {
          try {
            const jsonStr = line.substring(6);
            const data: ChatMessageData = JSON.parse(jsonStr);
            onMessage(data);
          } catch (e) {
            console.error('Failed to parse JSON from SSE line:', line, e);
          }
        }
      }
    }
  } catch (error) {
    console.error('Stream chat response error:', error);
    onError(error as Event);
  }
}
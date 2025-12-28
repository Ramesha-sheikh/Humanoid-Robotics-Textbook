/**
 * API Client for RAG Chatbot Backend
 * Connects to FastAPI backend with streaming support
 */

// Use environment-based URL for backend
// For Docusaurus, you'll need to update this to your actual deployed backend URL when not running locally
let API_BASE_URL = 'http://localhost:8001'; // Default for local development

// Update this to your actual deployed backend URL
if (typeof window !== 'undefined' && window.location.hostname !== 'localhost' && window.location.hostname !== '127.0.0.1') {
  // Production URL - Hugging Face Space backend
  API_BASE_URL = 'https://rameesha12123214-hackathone.hf.space';
}

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
 * @param question - The user's question
 * @param selected_text - Optional selected text for context
 * @param token - Optional JWT token for authenticated personalization
 */
export async function sendChatMessage(question: string, selected_text?: string, token?: string): Promise<ChatResponse> {
  const requestBody = { question, selected_text };
  const headers: Record<string, string> = {
    'Content-Type': 'application/json',
  };

  // Add Authorization header if token is provided
  if (token) {
    headers['Authorization'] = `Bearer ${token}`;
  }

  const response = await fetch(`${API_BASE_URL}/chat`, {
    method: 'POST',
    headers,
    body: JSON.stringify(requestBody),
  });

  if (!response.ok) {
    throw new Error(`Chat API error: ${response.statusText}`);
  }

  return response.json();
}

/**
 * Send chat request using non-streaming endpoint (temporary workaround)
 * Simulates streaming by returning the full response at once
 * @param question - The user's question
 * @param selected_text - Optional selected text for context
 * @param token - Optional JWT token for authenticated personalization
 */
export async function* streamChatMessage(question: string, selected_text?: string, token?: string): AsyncGenerator<StreamChunk> {
  const requestBody = { question, selected_text };
  const headers: Record<string, string> = {
    'Content-Type': 'application/json',
  };

  // Add Authorization header if token is provided
  if (token) {
    headers['Authorization'] = `Bearer ${token}`;
  }

  try {
    // Use /chat endpoint
    const response = await fetch(`${API_BASE_URL}/chat`, {
      method: 'POST',
      headers,
      body: JSON.stringify(requestBody),
    });

    if (!response.ok) {
      const errorText = await response.text();
      throw new Error(`API error (${response.status}): ${errorText || response.statusText}`);
    }

    // Get the full response
    const chatResponse: ChatResponse = await response.json();

    // Check if database is empty
    if (chatResponse.answer.includes('No relevant data found')) {
      yield {
        done: true,
        error: '❌ Database is empty - The backend database needs to be populated with book content.\n\n🔧 Admin: Run the ingestion endpoint to populate the database:\ncurl -X POST https://rameesha12123214-hackathone.hf.space/admin/ingest'
      };
      return;
    }

    // Simulate streaming by yielding the answer word by word
    const words = chatResponse.answer.split(' ');
    for (let i = 0; i < words.length; i++) {
      const token = (i === 0 ? '' : ' ') + words[i];
      yield {
        token,
        done: false
      };
      // Small delay to simulate streaming (optional, can be removed for instant response)
      await new Promise(resolve => setTimeout(resolve, 20));
    }

    // Final chunk with complete response
    yield {
      done: true,
      response: chatResponse
    };
  } catch (error: any) {
    if (error.message.includes('Failed to fetch') || error.message.includes('NetworkError')) {
      yield {
        done: true,
        error: '🌐 Network error - Cannot connect to backend. Please check your internet connection and try again.'
      };
    } else {
      yield {
        done: true,
        error: error.message || 'An unexpected error occurred. Please try again.'
      };
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

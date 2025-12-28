// ============================================
// FRONTEND TRANSLATION CLIENT
// ============================================
// This file calls the BACKEND API (Cohere-based translation)
// Uses the same backend as chatbot (port 8001)

// Backend API URL - uses chatbot backend with Cohere translation
const API_URL = process.env.NODE_ENV === 'production'
  ? 'https://rameesha12123214-hackathone.hf.space' // Hugging Face Spaces
  : 'http://localhost:8001'; // Local backend

interface TranslationProgress {
  currentChunk: number;
  totalChunks: number;
  onProgress?: (progress: number) => void;
}

// Rate limiting: Track last request time (client-side protection)
let lastRequestTime = 0;
const MIN_REQUEST_INTERVAL = 2000; // 2 seconds between requests

// Sleep utility
const sleep = (ms: number) => new Promise(resolve => setTimeout(resolve, ms));

// Check cache with localStorage fallback
function getCachedTranslation(cacheKey: string): string | null {
  try {
    // Try sessionStorage first (temporary)
    if (typeof sessionStorage !== 'undefined') {
      const cached = sessionStorage.getItem(cacheKey);
      if (cached) {
        console.log("✅ Cache hit (sessionStorage)");
        return cached;
      }
    }

    // Fallback to localStorage (persistent)
    if (typeof localStorage !== 'undefined') {
      const cached = localStorage.getItem(cacheKey);
      if (cached) {
        console.log("✅ Cache hit (localStorage)");
        return cached;
      }
    }
  } catch (err) {
    console.warn("Cache read failed:", err);
  }

  return null;
}

// Save to cache with dual storage
function setCachedTranslation(cacheKey: string, translation: string): void {
  try {
    // Save to both storages
    if (typeof sessionStorage !== 'undefined') {
      sessionStorage.setItem(cacheKey, translation);
    }
    if (typeof localStorage !== 'undefined') {
      localStorage.setItem(cacheKey, translation);
      console.log("✅ Cached to persistent storage");
    }
  } catch (err) {
    console.warn("Cache write failed:", err);
  }
}

/**
 * Main translation function
 *
 * IMPORTANT: This function NO LONGER uses @google/generative-ai directly.
 * Instead, it calls the backend API server which handles Gemini API securely.
 *
 * @param markdown - The markdown content to translate
 * @param slug - Page identifier for caching
 * @param apiKey - DEPRECATED - Not used anymore (kept for compatibility)
 * @param onProgress - Optional callback for progress updates
 * @returns Promise with translated text
 */
export async function translateToUrdu(
  markdown: string,
  slug: string,
  apiKey: string, // DEPRECATED - Not used (backend has the key)
  onProgress?: (progress: number) => void
): Promise<string> {
  console.log("=== 🌍 Frontend Translation Client (Cohere Backend) ===");
  console.log("Backend API:", API_URL);
  console.log("Page:", slug);

  // v4 cache: now uses page URL instead of markdown content
  const cacheKey = `urdu_v4_${slug.replace(/[^a-zA-Z0-9]/g, '_')}`;

  // Check cache first
  const cached = getCachedTranslation(cacheKey);
  if (cached) {
    if (onProgress) onProgress(100);
    return cached;
  }

  // No need to validate markdown anymore - backend fetches content from Qdrant

  // Rate limiting check (client-side)
  const now = Date.now();
  const timeSinceLastRequest = now - lastRequestTime;
  if (timeSinceLastRequest < MIN_REQUEST_INTERVAL) {
    const waitTime = MIN_REQUEST_INTERVAL - timeSinceLastRequest;
    console.log(`⏳ Rate limiting: waiting ${waitTime}ms...`);
    await sleep(waitTime);
  }
  lastRequestTime = Date.now();

  try {
    console.log("📤 Sending request to backend API...");
    console.log("Current page URL:", window.location.href);

    if (onProgress) onProgress(10); // Initial progress

    // Call backend /translate endpoint (Cohere-based, fast!)
    // Only sends page URL, backend fetches relevant content from Qdrant
    const response = await fetch(`${API_URL}/translate`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        page_url: window.location.href, // Current page URL
        target_language: 'urdu'
      })
    });

    if (onProgress) onProgress(50); // Midway progress

    // Check if response is ok
    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      throw new Error(errorData.error || `Server error: ${response.status}`);
    }

    // Parse response
    const data = await response.json();

    if (!data.success) {
      throw new Error(data.error || 'Translation failed');
    }

    const translation = data.translation;
    console.log("✅ Translation received from backend!");
    console.log("Translated length:", translation.length, "chars");

    if (onProgress) onProgress(90);

    // Cache result (now cached by page URL, not markdown content)
    setCachedTranslation(cacheKey, translation);

    if (onProgress) onProgress(100);

    return translation;

  } catch (error: any) {
    console.error("\n❌ Translation Error:", error);

    // User-friendly error messages
    let userMessage = "Translation failed. Please try again.";

    if (error.message?.includes('Failed to fetch') || error.message?.includes('NetworkError')) {
      userMessage = "❌ Cannot connect to translation server.\n\n🔧 Fix:\n1. Check your internet connection\n2. Backend server might be down\n3. Try again later";
    } else if (error.message?.includes('No content found')) {
      userMessage = "❌ Page content not found in database.\n\n🔧 Database may be empty. Admin needs to run ingest script.";
    } else if (error.message?.includes('Rate limit')) {
      userMessage = "❌ Rate limit exceeded.\n\n⏱️ Please wait a moment and try again.";
    } else if (error.message?.includes('Cohere')) {
      userMessage = "❌ Translation service error.\n\n🔧 Backend API issue. Check server logs.";
    } else if (error.message) {
      userMessage = `❌ Error: ${error.message}`;
    }

    throw new Error(userMessage);
  }
}

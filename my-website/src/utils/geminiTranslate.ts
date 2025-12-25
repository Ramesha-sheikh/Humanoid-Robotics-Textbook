// ============================================
// FRONTEND TRANSLATION CLIENT
// ============================================
// This file calls the BACKEND API (no Gemini SDK here!)
// API key is NEVER exposed to the browser

// Backend API URL (change for production deployment)
const API_URL = process.env.NODE_ENV === 'production'
  ? 'https://your-backend-api.com' // TODO: Update for production
  : 'http://localhost:5000';

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
  console.log("=== 🌍 Frontend Translation Client (Backend API) ===");
  console.log("Content length:", markdown.length, "chars");
  console.log("Backend API:", API_URL);

  const cacheKey = `urdu_v3_${slug}`; // v3 for new backend system

  // Check cache first
  const cached = getCachedTranslation(cacheKey);
  if (cached) {
    if (onProgress) onProgress(100);
    return cached;
  }

  // Validate input
  if (!markdown || markdown.trim().length === 0) {
    throw new Error('⚠️ No content to translate');
  }

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

    if (onProgress) onProgress(10); // Initial progress

    // Call backend API using fetch()
    const response = await fetch(`${API_URL}/api/translate`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        markdown: markdown,
        slug: slug
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

    // Cache result
    setCachedTranslation(cacheKey, translation);

    if (onProgress) onProgress(100);

    return translation;

  } catch (error: any) {
    console.error("\n❌ Translation Error:", error);

    // User-friendly error messages
    let userMessage = "Translation failed. Please try again.";

    if (error.message?.includes('Failed to fetch') || error.message?.includes('NetworkError')) {
      userMessage = "❌ Cannot connect to translation server.\n\n🔧 Fix:\n1. Make sure backend server is running\n2. Run: cd my-website/backend && npm start\n3. Check that server is at http://localhost:5000";
    } else if (error.message?.includes('Invalid API key')) {
      userMessage = "❌ Server API key is invalid.\n\n🔧 Fix: Check backend .env file and restart server";
    } else if (error.message?.includes('Rate limit exceeded')) {
      userMessage = "❌ Rate limit exceeded.\n\n⏱️ Please wait 60 seconds and try again.\n📊 Free tier: 15 requests/minute";
    } else if (error.message?.includes('Access denied')) {
      userMessage = "❌ Access denied.\n\n🔧 Fix: Check API key permissions on server";
    } else if (error.message?.includes('Model') && error.message?.includes('not found')) {
      userMessage = "❌ Translation model not available.\n\n🔧 Fix: Check backend server configuration";
    } else if (error.message) {
      userMessage = `❌ Error: ${error.message}`;
    }

    throw new Error(userMessage);
  }
}

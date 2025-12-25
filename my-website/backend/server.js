// ============================================
// BACKEND TRANSLATION API SERVER
// ============================================
// This server runs SEPARATELY from Docusaurus
// It handles Gemini API calls SERVER-SIDE (secure)
// Frontend calls this API using fetch()

const express = require('express');
const cors = require('cors');
const { GoogleGenerativeAI } = require('@google/generative-ai');
require('dotenv').config();

const app = express();
const PORT = process.env.BACKEND_PORT || 5000;

// ============================================
// MIDDLEWARE
// ============================================

// Enable CORS for Docusaurus frontend (localhost:3000)
app.use(cors({
  origin: ['http://localhost:3000', 'http://localhost:3001', 'https://humanoid-robotics-textbook-psi.vercel.app'],
  methods: ['POST', 'GET'],
  credentials: true
}));

// Parse JSON request bodies
app.use(express.json({ limit: '10mb' })); // Allow large markdown content

// ============================================
// CONFIGURATION
// ============================================

// Read API key from environment (SECURE - not exposed to frontend)
const GEMINI_API_KEY = process.env.GEMINI_API_KEY;

if (!GEMINI_API_KEY) {
  console.error('❌ ERROR: GEMINI_API_KEY not found in environment variables');
  console.error('Please create a .env file with: GEMINI_API_KEY=your_key_here');
  process.exit(1);
}

// Translation prompt (preserved from original logic)
const TRANSLATION_PROMPT = `Translate the following Markdown content to natural, fluent Urdu.
Preserve ALL formatting exactly: headings (# ## ###), code blocks, lists, tables, links, images, bold/italic.
Do not add, remove, or summarize any content. Only translate text portions.
Keep code inside \`\`\` blocks untouched.

Markdown content:
`;

// Model configuration (using stable model compatible with v1beta API)
const MODEL_NAME = 'gemini-pro';

// Chunking configuration
const CHUNK_SIZE = 4000; // Characters per chunk
const MAX_RETRIES = 3;
const RETRY_DELAY = 1000; // 1 second

// ============================================
// HELPER FUNCTIONS
// ============================================

// Sleep utility for retry mechanism
const sleep = (ms) => new Promise(resolve => setTimeout(resolve, ms));

// Split content into chunks at paragraph boundaries
function splitIntoChunks(content, chunkSize) {
  const chunks = [];
  const paragraphs = content.split(/\n\n+/);
  let currentChunk = '';

  for (const para of paragraphs) {
    if (currentChunk.length + para.length > chunkSize && currentChunk.length > 0) {
      chunks.push(currentChunk.trim());
      currentChunk = para;
    } else {
      currentChunk += (currentChunk ? '\n\n' : '') + para;
    }
  }

  if (currentChunk.trim()) {
    chunks.push(currentChunk.trim());
  }

  return chunks;
}

// Translate single chunk with retry logic
async function translateChunkWithRetry(content, retryCount = 0) {
  try {
    console.log(`  📤 Sending chunk to Gemini API (${content.length} chars)...`);

    // Initialize Gemini AI
    const genAI = new GoogleGenerativeAI(GEMINI_API_KEY);
    const model = genAI.getGenerativeModel({ model: MODEL_NAME });

    const fullPrompt = TRANSLATION_PROMPT + content;
    const result = await model.generateContent(fullPrompt);
    const translated = result.response.text();

    console.log(`  ✅ Translation successful (${translated.length} chars)`);
    return translated;

  } catch (error) {
    console.error(`  ❌ Translation error:`, error.message);

    // Retry logic for transient errors
    if (retryCount < MAX_RETRIES) {
      const isRetryable =
        error.message?.includes('429') ||
        error.message?.includes('503') ||
        error.message?.includes('timeout') ||
        error.message?.includes('DEADLINE_EXCEEDED');

      if (isRetryable) {
        const delay = RETRY_DELAY * Math.pow(2, retryCount); // Exponential backoff
        console.warn(`  ⚠️ Retrying in ${delay}ms (attempt ${retryCount + 1}/${MAX_RETRIES})...`);
        await sleep(delay);
        return translateChunkWithRetry(content, retryCount + 1);
      }
    }

    throw error;
  }
}

// ============================================
// API ROUTES
// ============================================

// Health check endpoint
app.get('/health', (req, res) => {
  res.json({
    status: 'ok',
    message: 'Translation API server is running',
    model: MODEL_NAME,
    timestamp: new Date().toISOString()
  });
});

// Main translation endpoint
app.post('/api/translate', async (req, res) => {
  const startTime = Date.now();
  console.log('\n=== 🌍 Translation Request Received ===');

  try {
    // Extract request body
    const { markdown, slug } = req.body;

    // Validate input
    if (!markdown || typeof markdown !== 'string') {
      console.error('❌ Invalid request: markdown is required');
      return res.status(400).json({
        error: 'Bad Request',
        message: 'markdown field is required and must be a string'
      });
    }

    console.log(`📝 Content length: ${markdown.length} chars`);
    console.log(`📋 Slug: ${slug || 'not provided'}`);

    // Split content into chunks
    const chunks = splitIntoChunks(markdown, CHUNK_SIZE);
    console.log(`📦 Content split into ${chunks.length} chunk(s)`);

    const translatedChunks = [];

    // Translate each chunk
    for (let i = 0; i < chunks.length; i++) {
      console.log(`\n📝 Translating chunk ${i + 1}/${chunks.length}...`);

      const translated = await translateChunkWithRetry(chunks[i]);
      translatedChunks.push(translated);

      // Send progress update (Server-Sent Events style)
      // Note: For simplicity, we're not using SSE here, but you can add it
      const progress = Math.round(((i + 1) / chunks.length) * 100);
      console.log(`📊 Progress: ${progress}%`);

      // Small delay between chunks to respect rate limits
      if (i < chunks.length - 1) {
        await sleep(1000);
      }
    }

    // Combine all translated chunks
    const fullTranslation = translatedChunks.join('\n\n');
    const duration = Date.now() - startTime;

    console.log(`\n✅ Translation complete!`);
    console.log(`⏱️ Duration: ${duration}ms`);
    console.log(`📏 Translated length: ${fullTranslation.length} chars`);

    // Send success response
    res.json({
      success: true,
      translation: fullTranslation,
      metadata: {
        originalLength: markdown.length,
        translatedLength: fullTranslation.length,
        chunks: chunks.length,
        duration: duration,
        model: MODEL_NAME,
        timestamp: new Date().toISOString()
      }
    });

  } catch (error) {
    console.error('\n❌ Translation failed:', error);

    // User-friendly error messages
    let statusCode = 500;
    let userMessage = 'Translation failed. Please try again.';

    if (error.message?.includes('API_KEY_INVALID') || error.message?.includes('API key not valid')) {
      statusCode = 401;
      userMessage = 'Invalid API key. Please check server configuration.';
    } else if (error.message?.includes('429') || error.message?.includes('quota') || error.message?.includes('RESOURCE_EXHAUSTED')) {
      statusCode = 429;
      userMessage = 'Rate limit exceeded. Please wait 60 seconds and try again.';
    } else if (error.message?.includes('403') || error.message?.includes('permission')) {
      statusCode = 403;
      userMessage = 'Access denied. Please check API key permissions.';
    } else if (error.message?.includes('404')) {
      statusCode = 404;
      userMessage = `Model "${MODEL_NAME}" not found. Please check server configuration.`;
    }

    res.status(statusCode).json({
      success: false,
      error: userMessage,
      details: process.env.NODE_ENV === 'development' ? error.message : undefined
    });
  }
});

// ============================================
// START SERVER
// ============================================

app.listen(PORT, () => {
  console.log('\n=================================================');
  console.log('🚀 Translation API Server Started!');
  console.log('=================================================');
  console.log(`📍 URL: http://localhost:${PORT}`);
  console.log(`🔧 Model: ${MODEL_NAME}`);
  console.log(`🔑 API Key: ${GEMINI_API_KEY ? '✅ Loaded' : '❌ Missing'}`);
  console.log(`🌍 CORS Enabled for: localhost:3000`);
  console.log('=================================================\n');
  console.log('📝 Available endpoints:');
  console.log(`   GET  /health           - Health check`);
  console.log(`   POST /api/translate    - Translate markdown to Urdu`);
  console.log('=================================================\n');
});

// Graceful shutdown
process.on('SIGTERM', () => {
  console.log('\n👋 SIGTERM received. Shutting down gracefully...');
  process.exit(0);
});

process.on('SIGINT', () => {
  console.log('\n👋 SIGINT received. Shutting down gracefully...');
  process.exit(0);
});

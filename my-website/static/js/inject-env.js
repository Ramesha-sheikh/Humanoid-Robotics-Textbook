// Inject environment variables to window object
(function() {
  // This script runs in the browser
  // The API key should be set via environment variables during build
  // For security, never commit your actual API key
  // Set GEMINI_API_KEY in your .env file or Vercel environment variables

  // For now, we'll leave it empty and you need to configure it
  window.GEMINI_API_KEY = '';

  console.log('Gemini API key configured:', window.GEMINI_API_KEY ? 'Yes' : 'No');
})();

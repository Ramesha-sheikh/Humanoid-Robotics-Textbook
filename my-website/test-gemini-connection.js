// Quick test script to verify Gemini API key
const { GoogleGenerativeAI } = require("@google/generative-ai");

// Directly use the API key from .env file
const API_KEY = "AIzaSyBfNQfMjCVul2uNuNE2YC6hjLbC_LWtFsQ";

console.log("=== Gemini API Test ===");
console.log("API Key available:", !!API_KEY);
console.log("API Key length:", API_KEY.length);
console.log("API Key starts with:", API_KEY.substring(0, 10) + "...");

async function testGeminiAPI() {
  try {
    const genAI = new GoogleGenerativeAI(API_KEY);
    const model = genAI.getGenerativeModel({ model: "gemini-2.0-flash-exp" });

    console.log("\n📤 Testing gemini-2.0-flash-exp...");

    const result = await model.generateContent("Translate 'Hello World' to Urdu.");
    const text = result.response.text();

    console.log("✅ API Key is VALID!");
    console.log("✅ Model: gemini-2.0-flash-exp is accessible");
    console.log("✅ Response:", text);
    console.log("\n🎉 Translation feature should work!");

  } catch (error) {
    console.error("\n❌ API Test Failed:");
    console.error("Error message:", error.message);
    console.error("Error status:", error.status || error.statusCode || "N/A");

    if (error.message.includes('API_KEY_INVALID') || error.message.includes('API key not valid')) {
      console.error("\n🔴 Your API key is INVALID or EXPIRED");
      console.error("👉 Generate new key at: https://aistudio.google.com/apikey");
    } else if (error.message.includes('404')) {
      console.error("\n🔴 Model not found. Your API key may not have access to Gemini 2.0");
      console.error("👉 Try generating a new API key at: https://aistudio.google.com/apikey");
    } else if (error.message.includes('429') || error.message.includes('quota')) {
      console.error("\n🟡 Rate limit exceeded. Wait 60 seconds and try again.");
    }
  }
}

testGeminiAPI();

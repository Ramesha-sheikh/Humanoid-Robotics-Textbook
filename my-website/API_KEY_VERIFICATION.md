# API Key Verification & Quota Troubleshooting

## Current Issue: API Quota Exceeded Error

Yeh error 3 main reasons se aa sakta hai:

### Reason 1: API Key Invalid Ya Expired
**Solution:**
1. [Google AI Studio](https://aistudio.google.com/app/apikey) par jayen
2. Login karein
3. Apni API keys list check karein
4. Current key "Active" status mein hai verify karein
5. Agar inactive hai to DELETE karein aur naya key generate karein

### Reason 2: Gemini 2.0-Flash Access Nahi Hai
**Problem:** Gemini 2.0 Flash experimental model hai, har account ko access nahi hai.

**Check Kaise Karein:**
1. [Google AI Studio](https://aistudio.google.com/app/prompts/new_chat) par jayen
2. Chat interface kholein
3. Model selector dropdown check karein
4. Dekhen ke "Gemini 2.0 Flash" available hai ya nahi

**Solutions:**

#### Option A: Use Gemini 1.5 Pro (Stable)
Agar 2.0 Flash access nahi hai, to stable model use karein:
- Model: `gemini-1.5-pro`
- Most reliable
- Free tier available

#### Option B: Use Gemini 1.5 Flash (Fast)
- Model: `gemini-1.5-flash-latest`
- Faster than Pro
- Good quality

### Reason 3: Actual Quota Limit

**Free Tier Limits:**
- **Gemini 1.5 Pro**: 2 RPM (requests per minute)
- **Gemini 1.5 Flash**: 15 RPM
- **Gemini 2.0 Flash**: Limited access, varies by account

**Check Quota:**
1. [Google Cloud Console](https://console.cloud.google.com/apis/api/generativelanguage.googleapis.com/quotas) par jayen
2. Generative Language API quotas dekhen
3. Current usage check karein

## Recommended Fix: Try Gemini 1.5 Pro

Main aapke liye code change kar deta hoon stable model ke liye:

```typescript
// Option 1: Gemini 1.5 Pro (Most Stable)
const model = genAI.getGenerativeModel({ model: "gemini-1.5-pro" });

// Option 2: Gemini 1.5 Flash (Faster)
const model = genAI.getGenerativeModel({ model: "gemini-1.5-flash-latest" });

// Option 3: Gemini 2.0 Flash (If you have access)
const model = genAI.getGenerativeModel({ model: "gemini-2.0-flash-exp" });
```

## Step-by-Step Verification

### Step 1: Verify API Key

Run this test:
1. Go to: https://aistudio.google.com/app/apikey
2. Check your key is "Active"
3. Copy the key
4. Update `.env` file:
   ```
   GEMINI_API_KEY="your_verified_key_here"
   ```

### Step 2: Check Model Access

```bash
# Test karein ke konsa model available hai
# Console logs se pata chalega exact error
```

### Step 3: Simple Test

1. Google AI Studio par jayen: https://aistudio.google.com/app/prompts/new_chat
2. Manually test karein: "Translate 'hello' to Urdu"
3. Dekhen ke kaunsa model kaam kar raha hai
4. Same model code mein use karein

## Quick Commands

```bash
# 1. Update .env with verified key
cd my-website
notepad .env

# 2. Restart server
npm start

# 3. Clear browser cache
# Press F12 > Application > Clear Storage > Clear site data
```

## Error Messages Decoded

| Error | Meaning | Solution |
|-------|---------|----------|
| "quota exceeded" | Model access issue OR rate limit | Try different model |
| "API key invalid" | Wrong/expired key | Regenerate key |
| "model not found" | Model name wrong OR no access | Check model availability |
| "404" | Model doesn't exist for API version | Use stable model name |

## Contact Info

Agar koi confusion ho to yeh info share karein:
1. Console error screenshot (F12)
2. API key first 10 characters (AIzaSyXXXX...)
3. Google AI Studio mein available models ki list

---

**Next Step:** Main aapke liye Gemini 1.5 Pro model set kar deta hoon (most stable).

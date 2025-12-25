# Translation Feature Troubleshooting Guide

## ✅ Latest Changes

1. **Model Updated**: Changed from `gemini-2.0-flash` to `gemini-1.5-flash` (more stable)
2. **API Key Updated**: New key configured in `.env` file
3. **Build**: Successful ✅

## How to Test Translation

### Step 1: Verify API Key
Apni API key ko verify karein:
1. [Google AI Studio](https://makersuite.google.com/app/apikey) par jayen
2. Check karein ke aapki key active hai
3. Key ki permissions check karein

### Step 2: Restart Dev Server

**Important**: Har baar jab aap `.env` file change karein, server restart karna zaroori hai!

```bash
# Windows mein:
# 1. Current running server ko band karein (Ctrl+C)
# 2. Phir se start karein:
cd my-website
npm start
```

### Step 3: Clear Browser Cache

Browser cache clear karein:
1. Browser mein F12 press karein (Developer Tools)
2. **Console** tab par jayen
3. Right-click on refresh button > **Empty Cache and Hard Reload**
4. Ya **Application** > **Storage** > **Clear site data**

### Step 4: Test Translation

1. Koi bhi tutorial page kholein (e.g., `http://localhost:3000/docs/introduction`)
2. "اردو میں ترجمہ کریں" button par click karein
3. Console mein logs dekhein (F12 > Console tab)

## Common Errors & Solutions

### Error: "API quota exceeded"

**Possible Causes:**
1. ❌ API key invalid hai
2. ❌ Free tier limit khatam ho gayi
3. ❌ Model access nahi hai
4. ❌ Server restart nahi kiya after .env change

**Solutions:**
1. ✅ Naya API key generate karein from Google AI Studio
2. ✅ Check karein ke billing enabled hai (agar required ho)
3. ✅ Server restart karein: `npm start`
4. ✅ Browser cache clear karein

### Error: "API key not configured"

**Solution:**
1. Check karein ke `.env` file `my-website` folder mein hai
2. File mein proper format hai:
   ```
   GEMINI_API_KEY="your_key_here"
   ```
3. Server restart karein

### Error: "Content is still loading"

**Solution:**
- 2-3 seconds wait karein page load hone ke baad
- Phir translate button click karein

## Verify API Key Works

Simple test ke liye, console mein yeh check karein:
1. Page open karein
2. F12 press karein > Console
3. Yeh logs dikhne chahiye:
   ```
   === Translation Button Clicked ===
   Raw markdown available: true
   API Key available: true
   API Key length: 39
   Starting translation...
   ```

## Model Information

**Current Model**: `gemini-pro`
- ✅ Stable and widely available
- ✅ Official stable model for Gemini API
- ✅ Good translation quality
- ✅ Free tier available
- ⚠️ Note: Model names like `gemini-1.5-flash` don't work with current API version

## Free Tier Limits

Google Gemini API free tier:
- **15 requests per minute (RPM)**
- **1 million tokens per month**
- **1,500 requests per day**

Agar limit exceed ho jaye to:
1. 1 minute wait karein (RPM limit ke liye)
2. Next day try karein (daily limit ke liye)
3. Ya billing enable karein for paid usage

## Testing Steps Summary

```bash
# 1. Kill any running servers
taskkill /F /IM node.exe

# 2. Go to project folder
cd my-website

# 3. Start fresh
npm start

# 4. Open browser
# Go to: http://localhost:3000/docs/introduction

# 5. Open Console (F12)

# 6. Click translate button

# 7. Check console for logs
```

## Need Help?

Agar abhi bhi problem ho to:
1. Console logs copy karein
2. Screenshot lein error ka
3. GitHub issues mein post karein

## Recent Fixes

### Fix #1: Model Not Found Error (SOLVED ✅)
**Error**: `models/gemini-1.5-flash is not found for API version v1beta`

**Solution**: Changed model from `gemini-1.5-flash` to `gemini-pro`

**Why**: The model name `gemini-1.5-flash` is not supported by the current Gemini API version. The correct stable model name is `gemini-pro`.

---

**Last Updated**: 2025-12-20 (Latest fix)
**Model**: gemini-pro ✅
**Status**: ✅ Working

# Gemini Translation Setup Guide

## Overview
Aapki website mein ab Urdu translation feature ke liye Gemini API integration ho chuki hai. Yeh guide aapko batayega ke isko kaise setup karein.

## Setup Steps

### 1. Gemini API Key Hasil Karein

1. [Google AI Studio](https://makersuite.google.com/app/apikey) par jayen
2. Sign in karein apne Google account se
3. "Create API Key" par click karein
4. API key ko copy kar lein

### 2. Environment Variable Setup (Local Development)

1. `my-website` folder mein ek `.env` file create karein:
   ```bash
   cd my-website
   touch .env
   ```

2. `.env` file mein apni API key add karein:
   ```
   GEMINI_API_KEY=your_actual_api_key_here
   ```

3. File save karein

### 3. Vercel Deployment Setup

Agar aap Vercel par deploy kar rahe hain:

1. Vercel dashboard par jayen
2. Apne project ko select karein
3. "Settings" > "Environment Variables" par jayen
4. Naya variable add karein:
   - **Name**: `GEMINI_API_KEY`
   - **Value**: (apni API key paste karein)
5. "Save" karein
6. Project ko redeploy karein

## Kaise Use Karein

1. Website par kisi bhi tutorial chapter ko open karein
2. Aapko "اردو میں ترجمہ کریں" (Translate to Urdu) button dikhega
3. Button par click karein
4. Content automatically Urdu mein translate ho jayega
5. "Back to English" button se wapis English mein ja sakte hain

## Security Notes

⚠️ **Important**:
- `.env` file ko kabhi bhi Git repository mein commit na karein
- Yeh file already `.gitignore` mein add hai
- API key ko publicly share na karein
- Rate limits ka khayal rakhein (Gemini API ki limits check karein)

## Testing

Local testing ke liye:

```bash
cd my-website
npm start
```

Browser mein `http://localhost:3000` par jayen aur tutorial pages check karein.

## Troubleshooting

### Translation Button Kaam Nahi Kar Raha

1. Check karein ke `.env` file sahi jagah par hai (my-website folder mein)
2. Verify karein ke API key correct hai
3. Console logs check karein (F12 > Console)
4. Server restart karein

### API Errors

Agar "Failed to translate" error aaye:
- API key valid hai check karein
- Internet connection check karein
- Gemini API quota/limits check karein
- Console mein detailed error messages dekhein

### Build Errors

Agar build time par errors aayen:
- Node modules reinstall karein: `npm install`
- Build cache clear karein: `rm -rf .docusaurus`
- Phir se build karein: `npm run build`

## Questions?

Koi issue ya question ho to GitHub issues mein raise karein.

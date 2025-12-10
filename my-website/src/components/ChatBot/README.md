# ChatBot Component 🤖

RAG Chatbot UI for Humanoid Robotics Textbook powered by Gemini 2.0 Flash.

## 📁 Structure

```
ChatBot/
├── index.tsx                      # Main export (ChatBot + FloatingButton)
├── ChatWindow.tsx                 # Main chat interface
├── ChatWindow.module.css         # ChatWindow styles
├── ChatMessage.tsx                # Message bubble component
├── ChatMessage.module.css        # Message styles
├── ChatInput.tsx                  # Input field component
├── ChatInput.module.css          # Input styles
├── FloatingChatButton.tsx        # Floating action button
├── FloatingChatButton.module.css # Button styles
├── api.ts                         # Backend API client
└── README.md                      # This file
```

## 🎯 Features

✅ **Floating Chat Button** - Always visible at bottom-right
✅ **Streaming Responses** - Real-time token-by-token streaming
✅ **Source Citations** - Shows chapter, section, and page references
✅ **Mobile Responsive** - Works on phones, tablets, and desktop
✅ **Dark Mode Support** - Adapts to Docusaurus theme
✅ **Normal & Highlight Modes** - Query full book or selected text
✅ **Beautiful UI** - Modern, clean design with animations

## 🚀 How It Works

### 1. User clicks floating button (💬)
- Opens ChatWindow modal overlay
- Shows welcome message from assistant

### 2. User types question and hits Enter
- Message sent to backend at `http://localhost:8000/chat/stream`
- Backend uses Gemini 2.0 Flash to generate response
- Tokens streamed back via Server-Sent Events (SSE)

### 3. Response appears token-by-token
- Smooth streaming animation
- Source citations displayed below message
- User can send follow-up questions

## 🔧 Configuration

### Environment Variables

Add to `.env.local`:

```env
NEXT_PUBLIC_API_URL=http://localhost:8000
```

### Backend Requirements

ChatBot connects to FastAPI backend at `/chat/stream` endpoint.

**Expected Response Format (SSE):**

```
data: {"token": "Hello", "done": false}
data: {"token": " world", "done": false}
data: {"token": "", "done": true}
```

## 📱 Mobile Optimization

- **Touch targets**: Minimum 44x44px for iOS/Android
- **Keyboard handling**: Input stays visible when keyboard opens
- **Responsive layout**: Full-width on mobile, centered on desktop
- **Font sizes**: 16px minimum to prevent iOS zoom

## 🎨 Styling

All components use CSS Modules for scoped styling:

- `ChatWindow.module.css` - Main window and overlay
- `ChatMessage.module.css` - Message bubbles and sources
- `ChatInput.module.css` - Input field and send button
- `FloatingChatButton.module.css` - Floating action button

**CSS Variables Used:**
- `--ifm-color-primary` - Primary brand color
- `--ifm-color-emphasis-*` - Grayscale palette
- `--ifm-background-*` - Background colors
- `--ifm-font-*` - Typography

## 🧪 Testing

### Manual Testing Checklist

- [ ] Floating button appears at bottom-right
- [ ] Click button opens chat window
- [ ] Can type message and press Enter to send
- [ ] Message appears in chat history
- [ ] Response streams token-by-token
- [ ] Source citations appear below response
- [ ] Click X button closes chat
- [ ] Works on mobile (test on real device)
- [ ] Dark mode switch works
- [ ] Backend connection error shows user-friendly message

### Test with Mock Backend

If backend isn't running, ChatBot will show:
```
❌ Connection error: Failed to fetch.
Make sure the backend is running at http://localhost:8000
```

## 📚 API Client (`api.ts`)

### Functions

**`sendChatMessage(request)`** - Send non-streaming request
```typescript
const response = await sendChatMessage({
  query: "What is inverse kinematics?",
  mode: "normal"
});
```

**`streamChatMessage(request)`** - Stream response tokens
```typescript
for await (const chunk of streamChatMessage({
  query: "Explain bipedal locomotion",
  mode: "normal"
})) {
  console.log(chunk.token);
  if (chunk.done) break;
}
```

**`checkHealth()`** - Check backend health
```typescript
const health = await checkHealth();
console.log(health.status); // "ok"
```

## 🐛 Troubleshooting

### "Connection error: Failed to fetch"
- **Cause**: Backend not running
- **Fix**: Start backend with `cd backend/app && python main.py`

### Chatbot button not visible
- **Cause**: Root.tsx not configured
- **Fix**: Check `src/theme/Root.tsx` includes `<ChatBot />`

### Styles not applying
- **Cause**: CSS Modules not loaded
- **Fix**: Restart Docusaurus dev server (`npm start`)

### Mobile keyboard covers input
- **Cause**: Viewport not adjusting
- **Fix**: Already handled in ChatInput.module.css with proper heights

## 🔄 Integration with Docusaurus

ChatBot is added via theme swizzling in `src/theme/Root.tsx`:

```tsx
import ChatBot from '../components/ChatBot';

export default function Root({ children }) {
  return (
    <>
      {children}
      <ChatBot />
    </>
  );
}
```

This ensures ChatBot appears on **every page** of the textbook.

## 🚀 Future Enhancements

- [ ] Highlight Mode implementation (text selection)
- [ ] Chat history persistence (Neon Postgres)
- [ ] Voice input/output
- [ ] Copy message button
- [ ] Share chat feature
- [ ] Export chat to PDF
- [ ] Multi-language support
- [ ] Keyboard shortcuts (Ctrl+K to open)

## 📄 License

Part of Humanoid Robotics Textbook project (GIAIC 2025 Hackathon).

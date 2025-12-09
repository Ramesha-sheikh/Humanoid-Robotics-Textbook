import React from 'react';
import type { PropsWithChildren } from 'react';
import ChatBubble from '@docusaurus-chat-plugin/components/BookChatBot/ChatBubble';

// Default Docusaurus Root component (passed as children)
function Root({ children }: PropsWithChildren): JSX.Element {
  return (
    <>
      {children}
      <ChatBubble />
    </>
  );
}

export default Root;
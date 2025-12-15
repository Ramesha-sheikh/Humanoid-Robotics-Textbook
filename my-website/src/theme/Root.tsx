/**
 * Root Component - Docusaurus Theme Wrapper
 * This wraps the entire app and adds global components like ChatBot
 */

import React from 'react';
import RagChatbot from '../components/RagChatbot';

export default function Root({ children }): JSX.Element {
  return (
    <>
      {children}
      <RagChatbot />
    </>
  );
}

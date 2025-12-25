/**
 * Root Component - Docusaurus Theme Wrapper
 * This wraps the entire app and adds global components like ChatBot and Language Toggle
 */

import React from 'react';
import RagChatbot from '../components/RagChatbot';
import LanguageToggle from '../components/LanguageToggle';

export default function Root({ children }): JSX.Element {
  return (
    <>
      {children}
      <RagChatbot />
      <LanguageToggle />
    </>
  );
}

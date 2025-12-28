/**
 * Root Component - Docusaurus Theme Wrapper
 * This wraps the entire app and adds global components like ChatBot, Language Toggle, and Auth
 */

import React from 'react';
import { AuthProvider } from '../components/Auth/AuthContext';
import RagChatbot from '../components/RagChatbot';
import LanguageToggle from '../components/LanguageToggle';

export default function Root({ children }): JSX.Element {
  return (
    <AuthProvider>
      {children}
      <RagChatbot />
      <LanguageToggle />
    </AuthProvider>
  );
}

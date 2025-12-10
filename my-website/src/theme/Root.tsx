/**
 * Root Component - Docusaurus Theme Wrapper
 * This wraps the entire app and adds global components like ChatBot
 */

import React from 'react';
// import ChatBot from '../components/ChatBot';

export default function Root({ children }): JSX.Element {
  return (
    <>
      {children}
      {/* <ChatBot /> */}
    </>
  );
}

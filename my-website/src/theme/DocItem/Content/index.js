import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import {ThemeClassNames} from '@docusaurus/theme-common';
import {useDoc} from '@docusaurus/plugin-content-docs/client';
import Heading from '@theme/Heading';
import MDXContent from '@theme/MDXContent';
/**
 Title can be declared inside md content or declared through
 front matter and added manually. To make both cases consistent,
 the added title is added under the same div.markdown block
 See https://github.com/facebook/docusaurus/pull/4882#issuecomment-853021120

 We render a "synthetic title" if:
 - user doesn't ask to hide it with front matter
 - the markdown content does not already contain a top-level h1 heading
*/
function useSyntheticTitle() {
  const {metadata, frontMatter, contentTitle} = useDoc();
  const shouldRender =
    !frontMatter.hide_title && typeof contentTitle === 'undefined';
  if (!shouldRender) {
    return null;
  }
  return metadata.title;
}
export default function DocItemContent({children}) {
  const syntheticTitle = useSyntheticTitle();
  const { metadata } = useDoc();
  const [translatedContent, setTranslatedContent] = useState(null);
  const [isTranslated, setIsTranslated] = useState(false);
  const [isTranslating, setIsTranslating] = useState(false);

  useEffect(() => {
    // Check if user has Urdu preference when page loads
    // Only translate THIS page, not all pages
    const preferredLanguage = localStorage.getItem('preferredLanguage');
    if (preferredLanguage === 'ur') {
      handleTranslateToUrdu();
    }
  }, [metadata.id]);

  const handleTranslateToUrdu = async () => {
    // Check if already translated
    const cachedTranslation = sessionStorage.getItem(`translation_${metadata.id}`);
    if (cachedTranslation) {
      setTranslatedContent(cachedTranslation);
      setIsTranslated(true);
      return;
    }

    setIsTranslating(true);
    try {
      // Get current page URL
      const currentPageUrl = window.location.href;

      console.log('🌍 Translating using Cohere embeddings...');
      console.log('Page URL:', currentPageUrl);

      // Determine backend URL (same logic as api.ts)
      const API_BASE_URL = (typeof window !== 'undefined' && window.location.hostname !== 'localhost' && window.location.hostname !== '127.0.0.1')
        ? 'https://rameesha12123214-hackathone.hf.space'
        : 'http://localhost:8001';

      // Call Cohere translation backend (using embeddings)
      const response = await fetch(`${API_BASE_URL}/translate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          page_url: currentPageUrl,
          target_language: 'urdu'
        })
      });

      if (!response.ok) {
        throw new Error(`Translation failed: ${response.status}`);
      }

      const data = await response.json();

      if (!data.success) {
        throw new Error(data.error || 'Translation failed');
      }

      const translated = data.translation;
      console.log('✅ Translation received from Cohere embeddings!');

      setTranslatedContent(translated);
      setIsTranslated(true);
      sessionStorage.setItem(`translation_${metadata.id}`, translated);
    } catch (error) {
      console.error('❌ Translation error:', error);
      alert('Translation failed. Please make sure:\n1. Cohere backend is running (port 8001)\n   Run: cd rag-backend/chatbot && python -m uvicorn app:app --port 8001\n\nError: ' + error.message);
    } finally {
      setIsTranslating(false);
    }
  };

  if (isTranslating) {
    return (
      <div className={clsx(ThemeClassNames.docs.docMarkdown, 'markdown')}>
        <div style={{
          padding: '60px 40px',
          textAlign: 'center',
          fontSize: '16px',
          color: 'var(--ifm-color-secondary)',
          minHeight: '400px',
          display: 'flex',
          flexDirection: 'column',
          justifyContent: 'center',
          alignItems: 'center'
        }}>
          <div style={{
            marginBottom: '20px',
            fontSize: '48px',
            animation: 'pulse 1.5s ease-in-out infinite'
          }}>
            📚
          </div>
          <div style={{
            fontSize: '18px',
            fontWeight: '500',
            marginBottom: '10px',
            direction: 'rtl'
          }}>
            اردو میں ترجمہ ہو رہا ہے...
          </div>
          <div style={{
            fontSize: '14px',
            opacity: '0.7'
          }}>
            Translating to Urdu using Cohere AI...
          </div>
          <div style={{
            marginTop: '20px',
            padding: '8px 16px',
            background: 'var(--ifm-color-primary-lightest)',
            borderRadius: '20px',
            fontSize: '12px',
            color: 'var(--ifm-color-primary-darker)'
          }}>
            Please wait a few seconds
          </div>
        </div>
      </div>
    );
  }

  if (isTranslated && translatedContent) {
    return (
      <div className={clsx(ThemeClassNames.docs.docMarkdown, 'markdown')}>
        <div style={{
          whiteSpace: 'pre-wrap',
          lineHeight: '1.8',
          direction: 'rtl',
          textAlign: 'right',
          fontFamily: 'system-ui, -apple-system, sans-serif'
        }}>
          {translatedContent}
        </div>
      </div>
    );
  }

  return (
    <div className={clsx(ThemeClassNames.docs.docMarkdown, 'markdown')}>
      {syntheticTitle && (
        <header>
          <Heading as="h1">{syntheticTitle}</Heading>
        </header>
      )}
      <MDXContent>{children}</MDXContent>
    </div>
  );
}

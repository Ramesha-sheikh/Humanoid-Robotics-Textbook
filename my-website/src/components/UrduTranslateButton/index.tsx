import React, { useState, useEffect } from 'react';
import { translateToUrdu } from '../../utils/geminiTranslate';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

interface UrduTranslateButtonProps {
  slug?: string;
  originalMarkdown?: string;
}

const UrduTranslateButton: React.FC<UrduTranslateButtonProps> = ({ slug, originalMarkdown: propOriginalMarkdown }) => {
  const { siteConfig } = useDocusaurusContext();
  const apiKey = (siteConfig.customFields?.geminiApiKey as string) || '';

  const [rawMarkdown, setRawMarkdown] = useState<string | null>(propOriginalMarkdown || null);
  const [translatedContent, setTranslatedContent] = useState<string | null>(null);
  const [isTranslating, setIsTranslating] = useState<boolean>(false);
  const [showTranslated, setShowTranslated] = useState<boolean>(false);
  const [error, setError] = useState<string | null>(null);
  const [progress, setProgress] = useState<number>(0);
  const [copySuccess, setCopySuccess] = useState<boolean>(false);

  useEffect(() => {
    if (propOriginalMarkdown) {
      setRawMarkdown(propOriginalMarkdown);
      console.log("UrduTranslateButton: Content loaded, length:", propOriginalMarkdown.length);
    }
  }, [propOriginalMarkdown]);

  const handleTranslateClick = async () => {
    console.log("=== 🌍 Real-World Translation Started ===");
    console.log("Content available:", !!rawMarkdown);
    console.log("API Key available:", !!apiKey);

    if (!rawMarkdown) {
      setError("⏳ Content is still loading. Please wait a moment and try again.");
      return;
    }

    if (!apiKey) {
      setError("🔑 Gemini API key not configured. Please check your environment variables.");
      return;
    }

    setIsTranslating(true);
    setError(null);
    setProgress(0);

    try {
      console.log("🚀 Starting translation with fallback system...");
      const translated = await translateToUrdu(
        rawMarkdown,
        slug || window.location.pathname,
        apiKey,
        (progressValue) => {
          setProgress(Math.round(progressValue));
          console.log(`📊 Progress: ${Math.round(progressValue)}%`);
        }
      );

      console.log("✅ Translation successful!");
      setTranslatedContent(translated);
      setShowTranslated(true);
      setProgress(100);
    } catch (err: any) {
      console.error("❌ Translation error:", err);
      setError(err.message || 'Unknown error occurred');
    } finally {
      setIsTranslating(false);
      setTimeout(() => setProgress(0), 2000); // Reset progress after 2 seconds
    }
  };

  const handleToggleLanguage = () => {
    setShowTranslated(prev => !prev);
  };

  const handleCopyTranslation = async () => {
    if (!translatedContent) return;

    try {
      await navigator.clipboard.writeText(translatedContent);
      setCopySuccess(true);
      console.log("✅ Translation copied to clipboard");
      setTimeout(() => setCopySuccess(false), 2000);
    } catch (err) {
      console.error("❌ Copy failed:", err);
      setError("Failed to copy to clipboard. Please select and copy manually.");
    }
  };

  const handleDownloadTranslation = () => {
    if (!translatedContent) return;

    try {
      const blob = new Blob([translatedContent], { type: 'text/plain;charset=utf-8' });
      const url = URL.createObjectURL(blob);
      const link = document.createElement('a');
      link.href = url;

      // Create filename from slug or pathname
      const filename = (slug || window.location.pathname)
        .replace(/[^a-zA-Z0-9]/g, '_')
        .replace(/_+/g, '_') + '_urdu.txt';

      link.download = filename;
      document.body.appendChild(link);
      link.click();
      document.body.removeChild(link);
      URL.revokeObjectURL(url);

      console.log("✅ Translation downloaded:", filename);
    } catch (err) {
      console.error("❌ Download failed:", err);
      setError("Failed to download translation. Please try copying instead.");
    }
  };

  // Styles
  const containerStyle: React.CSSProperties = {
    marginBottom: '20px',
    padding: '15px',
    border: '2px solid #e0e0e0',
    borderRadius: '8px',
    backgroundColor: 'var(--ifm-background-surface-color)',
    boxShadow: '0 2px 4px rgba(0,0,0,0.1)'
  };

  const buttonGroupStyle: React.CSSProperties = {
    display: 'flex',
    gap: '10px',
    flexWrap: 'wrap',
    alignItems: 'center'
  };

  const progressBarStyle: React.CSSProperties = {
    marginTop: '10px',
    height: '8px',
    backgroundColor: '#e0e0e0',
    borderRadius: '4px',
    overflow: 'hidden',
    position: 'relative'
  };

  const progressFillStyle: React.CSSProperties = {
    height: '100%',
    backgroundColor: '#4CAF50',
    width: `${progress}%`,
    transition: 'width 0.3s ease',
    borderRadius: '4px'
  };

  const errorStyle: React.CSSProperties = {
    marginTop: '10px',
    padding: '12px',
    backgroundColor: '#fee',
    border: '1px solid #fcc',
    borderRadius: '5px',
    color: '#c00',
    whiteSpace: 'pre-wrap',
    fontSize: '14px'
  };

  const translatedBoxStyle: React.CSSProperties = {
    marginTop: '20px',
    padding: '20px',
    backgroundColor: 'var(--ifm-color-emphasis-100)',
    borderRadius: '8px',
    border: '2px solid var(--ifm-color-primary)',
    direction: 'rtl'
  };

  const translatedHeaderStyle: React.CSSProperties = {
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: '15px',
    direction: 'ltr'
  };

  const successMessageStyle: React.CSSProperties = {
    display: 'inline-block',
    marginLeft: '10px',
    padding: '4px 8px',
    backgroundColor: '#4CAF50',
    color: 'white',
    borderRadius: '4px',
    fontSize: '12px',
    fontWeight: 'bold'
  };

  return (
    <div className="urdu-translate-container" style={containerStyle}>
      <div style={buttonGroupStyle}>
        <button
          onClick={handleTranslateClick}
          disabled={isTranslating || !rawMarkdown}
          className="button button--primary"
          style={{ fontWeight: 'bold' }}
        >
          {isTranslating ? `ترجمہ ہو رہا ہے... ${progress}%` : '🌐 اردو میں ترجمہ کریں'}
        </button>

        {showTranslated && translatedContent && (
          <>
            <button
              onClick={handleToggleLanguage}
              className="button button--secondary"
            >
              {showTranslated ? '🔙 Back to English' : '📖 Show Translation'}
            </button>

            <button
              onClick={handleCopyTranslation}
              className="button button--info"
              style={{ backgroundColor: copySuccess ? '#4CAF50' : undefined }}
            >
              {copySuccess ? '✅ Copied!' : '📋 Copy Translation'}
            </button>

            <button
              onClick={handleDownloadTranslation}
              className="button button--success"
            >
              💾 Download
            </button>
          </>
        )}
      </div>

      {isTranslating && progress > 0 && (
        <div style={progressBarStyle}>
          <div style={progressFillStyle} />
        </div>
      )}

      {isTranslating && (
        <div style={{ marginTop: '10px', fontSize: '13px', color: 'var(--ifm-color-secondary)' }}>
          ⚡ Using multi-model fallback system for reliability...
        </div>
      )}

      {error && (
        <div style={errorStyle}>
          <strong>⚠️ Error:</strong> <br />
          {error}
        </div>
      )}

      {showTranslated && translatedContent && (
        <div style={translatedBoxStyle}>
          <div style={translatedHeaderStyle}>
            <h3 style={{ margin: 0 }}>📝 Urdu Translation</h3>
            {copySuccess && <span style={successMessageStyle}>Copied to clipboard!</span>}
          </div>
          <div style={{ whiteSpace: 'pre-wrap', lineHeight: '1.8', fontSize: '16px' }}>
            {translatedContent}
          </div>
          <div style={{
            marginTop: '15px',
            paddingTop: '15px',
            borderTop: '1px solid var(--ifm-color-emphasis-300)',
            fontSize: '12px',
            color: 'var(--ifm-color-secondary)',
            direction: 'ltr'
          }}>
            🤖 Translated using Google Gemini AI with multi-model fallback system
          </div>
        </div>
      )}
    </div>
  );
};

export default UrduTranslateButton;

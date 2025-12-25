import React, { useState, useEffect } from 'react';
import './styles.css';

const LanguageToggle: React.FC = () => {
  const [language, setLanguage] = useState<'en' | 'ur'>('en');

  // Load language preference from localStorage
  useEffect(() => {
    const savedLanguage = localStorage.getItem('preferredLanguage') as 'en' | 'ur';
    if (savedLanguage) {
      setLanguage(savedLanguage);
    }
  }, []);

  const handleLanguageToggle = async () => {
    const newLanguage = language === 'en' ? 'ur' : 'en';

    // Save preference and reload page
    // Only current page will translate, not all pages
    setLanguage(newLanguage);
    localStorage.setItem('preferredLanguage', newLanguage);

    // Reload to apply language change to current page only
    window.location.reload();
  };

  return (
    <div className="language-toggle-container">
      <button
        onClick={handleLanguageToggle}
        className={`language-toggle-btn ${language === 'ur' ? 'urdu-active' : ''}`}
        title={language === 'en' ? 'Switch to Urdu' : 'Switch to English'}
      >
        {language === 'en' ? (
          <>
            <span className="lang-icon">🌐</span>
            <span className="lang-text">اردو</span>
          </>
        ) : (
          <>
            <span className="lang-icon">🌐</span>
            <span className="lang-text">English</span>
          </>
        )}
      </button>
    </div>
  );
};

export default LanguageToggle;

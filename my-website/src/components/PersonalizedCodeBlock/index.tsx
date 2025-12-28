import React, { useState } from 'react';
import { useAuth } from '../Auth/AuthContext';
import CodeBlock from '@theme/CodeBlock';
import styles from './styles.module.css';

interface CodeExample {
  language: 'python' | 'cpp' | 'bash' | 'yaml';
  code: string;
  label?: string;
}

interface PersonalizedCodeBlockProps {
  pythonCode?: string;
  cppCode?: string;
  bashCode?: string;
  yamlCode?: string;
  title?: string;
  description?: string;
  showAllForBoth?: boolean; // If user knows both languages, show both by default
}

/**
 * Personalized Code Block Component
 * Shows code examples based on user's programming experience
 *
 * Usage:
 * <PersonalizedCodeBlock
 *   title="Creating a ROS Node"
 *   pythonCode={pythonExample}
 *   cppCode={cppExample}
 * />
 */
export default function PersonalizedCodeBlock({
  pythonCode,
  cppCode,
  bashCode,
  yamlCode,
  title,
  description,
  showAllForBoth = false,
}: PersonalizedCodeBlockProps): JSX.Element {
  const { user, isAuthenticated } = useAuth();

  // Determine user's programming preference
  const programmingExp = user?.programming_experience || 'None';

  // Determine default language based on user preference
  const getDefaultLanguage = (): 'python' | 'cpp' | 'bash' | 'yaml' => {
    if (!isAuthenticated) return pythonCode ? 'python' : cppCode ? 'cpp' : 'bash';

    if (programmingExp === 'Python' && pythonCode) return 'python';
    if (programmingExp === 'C++' && cppCode) return 'cpp';
    if (programmingExp === 'Both Python and C++') {
      // For users who know both, prefer Python unless they don't have it
      return pythonCode ? 'python' : cppCode ? 'cpp' : 'bash';
    }
    // For beginners or others, show Python first (more beginner-friendly)
    return pythonCode ? 'python' : cppCode ? 'cpp' : 'bash';
  };

  const [activeTab, setActiveTab] = useState<'python' | 'cpp' | 'bash' | 'yaml'>(
    getDefaultLanguage()
  );

  // Build available examples
  const examples: CodeExample[] = [];
  if (pythonCode) examples.push({ language: 'python', code: pythonCode, label: 'Python' });
  if (cppCode) examples.push({ language: 'cpp', code: cppCode, label: 'C++' });
  if (bashCode) examples.push({ language: 'bash', code: bashCode, label: 'Bash' });
  if (yamlCode) examples.push({ language: 'yaml', code: yamlCode, label: 'YAML' });

  // If only one example, show it directly without tabs
  if (examples.length === 1) {
    return (
      <div className={styles.codeBlockContainer}>
        {title && <h4 className={styles.codeTitle}>{title}</h4>}
        {description && <p className={styles.codeDescription}>{description}</p>}
        <CodeBlock language={examples[0].language}>{examples[0].code}</CodeBlock>
      </div>
    );
  }

  // Determine which tabs to show based on user preference
  const shouldShowTab = (lang: 'python' | 'cpp' | 'bash' | 'yaml'): boolean => {
    if (!isAuthenticated) return true; // Show all tabs if not authenticated

    // Always show bash and yaml
    if (lang === 'bash' || lang === 'yaml') return true;

    // For programming languages
    if (programmingExp === 'Python') {
      return lang === 'python' || (lang === 'cpp' && showAllForBoth);
    }
    if (programmingExp === 'C++') {
      return lang === 'cpp' || (lang === 'python' && showAllForBoth);
    }
    if (programmingExp === 'Both Python and C++') {
      return true; // Show all languages
    }
    // For beginners (None), show Python first but allow switching
    return true;
  };

  // Get personalization hint
  const getPersonalizationHint = (): string | null => {
    if (!isAuthenticated) return null;

    if (programmingExp === 'Python' && pythonCode) {
      return '✨ Python example shown based on your preference';
    }
    if (programmingExp === 'C++' && cppCode) {
      return '✨ C++ example shown based on your preference';
    }
    if (programmingExp === 'Both Python and C++') {
      return '✨ All examples available for your skill level';
    }
    if (programmingExp === 'None' && pythonCode) {
      return '💡 Python is recommended for beginners';
    }
    return null;
  };

  const hint = getPersonalizationHint();

  return (
    <div className={styles.codeBlockContainer}>
      {title && <h4 className={styles.codeTitle}>{title}</h4>}
      {description && <p className={styles.codeDescription}>{description}</p>}

      {hint && (
        <div className={styles.personalizationHint}>
          <span>{hint}</span>
        </div>
      )}

      <div className={styles.tabsContainer}>
        <div className={styles.tabs}>
          {examples.map((example) => (
            shouldShowTab(example.language) && (
              <button
                key={example.language}
                className={`${styles.tab} ${
                  activeTab === example.language ? styles.activeTab : ''
                }`}
                onClick={() => setActiveTab(example.language)}
              >
                {example.label || example.language}
                {activeTab === example.language && (
                  <span className={styles.activeIndicator}>●</span>
                )}
              </button>
            )
          ))}
        </div>

        <div className={styles.codeContent}>
          {examples.map(
            (example) =>
              activeTab === example.language && (
                <CodeBlock key={example.language} language={example.language}>
                  {example.code}
                </CodeBlock>
              )
          )}
        </div>
      </div>
    </div>
  );
}

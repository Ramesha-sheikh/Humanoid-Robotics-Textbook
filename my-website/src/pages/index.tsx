import type {ReactNode} from 'react';
import { useEffect } from 'react';
import { useHistory } from '@docusaurus/router';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';
import { useAuth } from '@site/src/components/Auth/AuthContext';

import styles from './index.module.css';

function HomepageHeader() {
  return (
    <header className={styles.heroBanner}>
      <div className={styles.bannerContainer}>
        <img
          src="/img/banner.png"
          alt="Physical AI & Humanoid Robotics"
          className={styles.bannerImage}
        />
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  const { isAuthenticated, isLoading } = useAuth();
  const history = useHistory();

  // Redirect to tutorial/docs if not authenticated
  useEffect(() => {
    if (!isLoading && !isAuthenticated) {
      // Redirect to docs introduction page
      history.push('/docs/introduction/');
    }
  }, [isAuthenticated, isLoading, history]);

  // Show loading while checking auth
  if (isLoading) {
    return (
      <Layout>
        <div style={{
          display: 'flex',
          justifyContent: 'center',
          alignItems: 'center',
          minHeight: '50vh'
        }}>
          <p>Loading...</p>
        </div>
      </Layout>
    );
  }

  // If not authenticated, don't render (will redirect)
  if (!isAuthenticated) {
    return null;
  }

  // Only show homepage if authenticated
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}

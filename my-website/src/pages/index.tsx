import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
<<<<<<< HEAD
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
=======
      <div className="container glass-card">
        <Heading as="h1" className="hero__title">
         Physical AI & Humanoid Robotics Book
        </Heading>
>>>>>>> a7d920193ce792bf5b2c8e211d93c297f7161419
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
<<<<<<< HEAD
            Docusaurus Tutorial - 5min ⏱️
=======
           "Learn Physical AI & Humanoid Robotics to work with intelligent agents and robots of the future."
>>>>>>> a7d920193ce792bf5b2c8e211d93c297f7161419
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
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

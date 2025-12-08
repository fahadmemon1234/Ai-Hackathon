import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';
import clsx from 'clsx'; // Standard Docusaurus utility

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={clsx('hero', 'heroBanner', 'hero--centered')}>
      <div className="container">
        {/* Premium Badge */}
        <div className="hero__badge">
          <span>New</span> v1.0 Released
        </div>

        {/* Title with Gradient Span */}
        <Heading as="h1" className="hero__title">
          The Guide to <br />
          <span className="text--gradient">{siteConfig.title}</span>
        </Heading>

        <p className="hero__subtitle">{siteConfig.tagline}</p>

        {/* Buttons: Primary (Gradient) & Secondary (Glass) */}
        <div className="buttons">
          <Link
            className="button button--primary button--lg glow-button"
            to="/docs/intro"
          >
            Get Started
          </Link>
          <Link
            className="button button--secondary button--lg glass-button"
            to="https://github.com/fahadmemon1234/Ai-Hackathon"
          >
            View on GitHub
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="A practical guide to building intelligent robots with Physical AI and Humanoid Robotics."
    >
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
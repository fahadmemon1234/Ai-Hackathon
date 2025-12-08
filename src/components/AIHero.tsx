import React from 'react';
import Link from '@docusaurus/Link';

export default function AIHero({ title, subtitle, ctaText, ctaLink }) {
  return (
    <div className="ai-hero">
      <h1 className="ai-hero__title">{title}</h1>
      <p className="ai-hero__subtitle">{subtitle}</p>
      <Link to={ctaLink} className="button button--primary button--lg">
        {ctaText}
      </Link>
    </div>
  );
}
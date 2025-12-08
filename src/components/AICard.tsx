import React from 'react';
import Link from '@docusaurus/Link';

export default function AICard({ to, title, children }) {
  return (
    <Link to={to} className="ai-card">
      <h3>{title}</h3>
      <p>{children}</p>
    </Link>
  );
}
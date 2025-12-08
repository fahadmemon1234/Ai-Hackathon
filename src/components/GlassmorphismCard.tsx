import React from 'react';
import Link from '@docusaurus/Link';

export default function GlassmorphismCard({ to, title, children }) {
  return (
    <Link to={to} className="glass-card">
      <h3>{title}</h3>
      <p>{children}</p>
    </Link>
  );
}

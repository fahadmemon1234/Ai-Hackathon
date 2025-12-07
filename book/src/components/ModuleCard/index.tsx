import React from 'react';

interface ModuleCardProps {
  children: React.ReactNode;
  title?: string;
  description?: string;
}

export default function ModuleCard({ children, title, description }: ModuleCardProps): JSX.Element {
  return (
    <div className="module-card">
      {title && <h2>{title}</h2>}
      {description && <p>{description}</p>}
      {children}
    </div>
  );
}
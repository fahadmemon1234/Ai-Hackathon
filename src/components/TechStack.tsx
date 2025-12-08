import React from 'react';

const technologies = [
  { name: 'React', logo: '.../react.svg' },
  { name: 'Docusaurus', logo: '.../docusaurus.svg' },
  { name: 'TypeScript', logo: '.../typescript.svg' },
  // Add more technologies here
];

export default function TechStack() {
  return (
    <div className="tech-stack">
      <h2 className="tech-stack__title">Powered By</h2>
      <div className="tech-stack__icons">
        {technologies.map((tech, i) => (
          <div className="tech-stack__icon" key={i}>
            <img src={tech.logo} alt={tech.name} />
            <span>{tech.name}</span>
          </div>
        ))}
      </div>
    </div>
  );
}

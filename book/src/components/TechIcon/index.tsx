import React from 'react';

interface TechIconProps {
  iconName: string; // e.g., 'ros2', 'gazebo', 'unity'
  alt: string;
  size?: number; // in px
}

export default function TechIcon({ iconName, alt, size = 24 }: TechIconProps): JSX.Element {
  const iconSrc = `/hackathon-book/img/icons/${iconName}.svg`; // Assuming base URL for icons
  return (
    <img
      src={iconSrc}
      alt={alt}
      style={{
        width: size,
        height: size,
        verticalAlign: 'middle',
        marginRight: '0.5em',
        display: 'inline-block',
      }}
    />
  );
}
import React, { useState } from 'react';
import Link from '@docusaurus/Link';
import { useThemeConfig } from '@docusaurus/theme-common';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export default function Navbar() {
  const { siteConfig } = useDocusaurusContext();
  const { navbar } = useThemeConfig();
  const [isMobileMenuOpen, setIsMobileMenuOpen] = useState(false);

  return (
    <nav className="navbar">
      <div className="navbar__inner">
        <div className="navbar__brand">
          <Link to="/" className="navbar__logo">
            <img src={navbar.logo.src} alt={navbar.logo.alt} />
          </Link>
          <Link to="/" className="navbar__title">
            {siteConfig.title}
          </Link>
        </div>
        <div className="navbar__items">
          {navbar.items.map((item, i) => (
            <Link key={i} to={item.to || item.href} className="navbar__item">
              {item.label}
            </Link>
          ))}
        </div>
        <div className="navbar__mobile-menu-button-container">
          <button
            className="navbar__mobile-menu-button"
            onClick={() => setIsMobileMenuOpen(!isMobileMenuOpen)}
          >
            {/* Hamburger Icon */}
            <svg width="24" height="24" viewBox="0 0 24 24">
              <path
                fill="currentColor"
                d="M3 18h18v-2H3v2zm0-5h18v-2H3v2zm0-7v2h18V6H3z"
              />
            </svg>
          </button>
        </div>
      </div>
      {isMobileMenuOpen && (
        <div className="navbar__mobile-menu">
          {navbar.items.map((item, i) => (
            <Link key={i} to={item.to || item.href} className="navbar__mobile-menu-item">
              {item.label}
            </Link>
          ))}
        </div>
      )}
    </nav>
  );
}

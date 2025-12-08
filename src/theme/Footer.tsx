import React from 'react';
import { useThemeConfig } from '@docusaurus/theme-common';
import Link from '@docusaurus/Link';

function Footer() {
  const { footer } = useThemeConfig();

  if (!footer) {
    return null;
  }

  const { copyright, links = [] } = footer;

  return (
    <footer className="footer">
      <div className="container">
        <div className="row">
          {links.map((linkItem, i) => (
            <div className="col" key={i}>
              <h4 className="footer__title">{linkItem.title}</h4>
              <ul className="footer__items">
                {linkItem.items.map((item, key) => (
                  <li className="footer__item" key={key}>
                    <Link
                      to={item.to || item.href}
                      className="footer__link-item"
                    >
                      {item.label}
                    </Link>
                  </li>
                ))}
              </ul>
            </div>
          ))}
        </div>
        <div className="footer__bottom text--center">
          <div className="footer__copyright">{copyright}</div>
        </div>
      </div>
    </footer>
  );
}

export default React.memo(Footer);

import React from "react";
import { Helmet } from "react-helmet";

export default function Root({ children }) {
  return (
    <>
      <Helmet>
        <link rel="preconnect" href="https://fonts.googleapis.com" />
        <link rel="preconnect" href="https://fonts.gstatic.com" crossOrigin="anonymous" />
        <link
          href="https://fonts.googleapis.com/css2?family=Inter:wght@400;700;800&display=swap"
          rel="stylesheet"
        />
      </Helmet>
      <div className="animated-gradient">{children}</div>
    </>
  );
}

import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import BrowserOnly from '@docusaurus/BrowserOnly';

const NotFoundPageContent = () => {
  const { siteConfig } = useDocusaurusContext();
  
  return (
    <div className="not-found-page">
      <div className="container margin-vert--xl">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <h1 className="hero__title">Page Not Found</h1>
            <p className="hero__subtitle">
              We couldn't find what you were looking for.
            </p>
            
            <div className="not-found-content">
              <p>
                The page you requested may have been moved, renamed, or might not exist.
              </p>
              
              <div className="not-found-actions">
                <div className="not-found-options">
                  <h3>Navigate to:</h3>
                  <ul>
                    <li>
                      <Link to="/">Homepage</Link>
                    </li>
                    <li>
                      <Link to="/signin">Sign In</Link>
                    </li>
                    <li>
                      <Link to="/signup">Sign Up</Link>
                    </li>
                    <li>
                      <Link to="/docs/intro">Textbook Introduction</Link>
                    </li>
                  </ul>
                </div>
                
                <div className="not-found-illustration">
                  <svg viewBox="0 0 200 200" width="200" height="200">
                    <circle cx="100" cy="80" r="40" fill="#f0f0f0" stroke="#ccc" strokeWidth="2" />
                    <circle cx="85" cy="70" r="5" fill="#333" />
                    <circle cx="115" cy="70" r="5" fill="#333" />
                    <path d="M85 110 Q100 125 115 110" stroke="#333" strokeWidth="3" fill="none" />
                    <rect x="50" y="120" width="100" height="60" rx="10" fill="#f0f0f0" stroke="#ccc" strokeWidth="2" />
                    <rect x="60" y="130" width="80" height="10" rx="2" fill="#e0e0e0" />
                    <rect x="60" y="145" width="60" height="10" rx="2" fill="#e0e0e0" />
                    <rect x="60" y="160" width="70" height="10" rx="2" fill="#e0e0e0" />
                  </svg>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

const NotFoundPage = () => {
  const { siteConfig } = useDocusaurusContext();
  
  return (
    <Layout title="Page Not Found" description="The page you are looking for doesn't exist">
      <BrowserOnly>
        {() => <NotFoundPageContent />}
      </BrowserOnly>
    </Layout>
  );
};

export default NotFoundPage;
import React from 'react';
import { useHistory } from '@docusaurus/router';
import Layout from '@theme/Layout';
import BrowserOnly from '@docusaurus/BrowserOnly';

const LogoutPageContent = () => {
  const history = useHistory();

  React.useEffect(() => {
    // Clear localStorage used for authentication state
    if (typeof window !== 'undefined') {
      localStorage.removeItem('mockUser');
      localStorage.removeItem('activeSession');
    }

    // Redirect to home page after logout
    history.push('/');
  }, [history]);

  return (
    <Layout title="Logging Out" description="Logging out of your account">
      <div style={{ padding: '2rem' }}>
        <div className="container">
          <h1>Logging Out</h1>
          <p>Please wait while we log you out...</p>
        </div>
      </div>
    </Layout>
  );
};

const LogoutPage = () => {
  return (
    <BrowserOnly>
      {() => <LogoutPageContent />}
    </BrowserOnly>
  );
};

export default LogoutPage;
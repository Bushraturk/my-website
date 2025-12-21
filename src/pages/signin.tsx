import React from 'react';
import Layout from '@theme/Layout';
import BrowserOnly from '@docusaurus/BrowserOnly';
import Translate from '@docusaurus/Translate';
import SigninForm from '../components/SigninForm';

const SigninPageContent = () => {
  return (
    <Layout title="Sign In" description="Sign in to your account">
      <div style={{ padding: '2rem' }}>
        <div className="container">
          <h1>
            <Translate id="signin.title">Sign In to Your Account</Translate>
          </h1>
          <SigninForm />
        </div>
      </div>
    </Layout>
  );
};

const SigninPage = () => {
  return (
    <BrowserOnly>
      {() => <SigninPageContent />}
    </BrowserOnly>
  );
};

export default SigninPage;

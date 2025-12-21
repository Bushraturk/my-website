import React from 'react';
import Layout from '@theme/Layout';
import BrowserOnly from '@docusaurus/BrowserOnly';
import Translate from '@docusaurus/Translate';
import SignupForm from '../components/SignupForm';

const SignupPageContent = () => {
  return (
    <Layout title="Sign Up" description="Create your account">
      <div style={{ padding: '2rem' }}>
        <div className="container">
          <h1>
            <Translate id="signup.title">Create Your Account</Translate>
          </h1>
          <SignupForm />
        </div>
      </div>
    </Layout>
  );
};

const SignupPage = () => {
  return (
    <BrowserOnly>
      {() => <SignupPageContent />}
    </BrowserOnly>
  );
};

export default SignupPage;

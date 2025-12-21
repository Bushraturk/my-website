import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import Translate from '@docusaurus/Translate';
import useBaseUrl from '@docusaurus/useBaseUrl';

const SigninFormContent = () => {
  const [email, setEmail] = React.useState('');
  const [password, setPassword] = React.useState('');
  const [error, setError] = React.useState('');
  const [loading, setLoading] = React.useState(false);
  const history = require('@docusaurus/router').useHistory();
  const profileUrl = useBaseUrl('/user-profile');

  // Define navigate function using history
  const navigate = (path) => {
    history.push(path);
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    setLoading(true);
    setError('');

    try {
      // Use the mock sign in function that properly checks user data
      const mockSignIn = require('../auth/client').mockSignIn;
      const result = await mockSignIn(email, password);

      if (result.error) {
        setError(result.error.message);
      } else {
        // Redirect to user profile page after successful sign in
        navigate(profileUrl);
      }
    } catch (err: any) {
      setError(err.message || 'An error occurred during sign in');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="signin-form-container">
      <h2><Translate id="signin.formTitle">Sign In to Your Account</Translate></h2>

      {error && <div className="alert alert--danger">{error}</div>}

      <form onSubmit={handleSubmit}>
        <div className="form-group">
          <label htmlFor="email"><Translate id="signin.email">Email</Translate>:</label>
          <input
            type="email"
            id="email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            required
          />
        </div>

        <div className="form-group">
          <label htmlFor="password"><Translate id="signin.password">Password</Translate>:</label>
          <input
            type="password"
            id="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            required
          />
        </div>

        <button type="submit" disabled={loading} className="button button--primary">
          {loading ? <Translate id="signin.signingIn">Signing In...</Translate> : <Translate id="signin.signInBtn">Sign In</Translate>}
        </button>
      </form>
    </div>
  );
};

const SigninForm = () => {
  return (
    <BrowserOnly>
      {() => <SigninFormContent />}
    </BrowserOnly>
  );
};

export default SigninForm;

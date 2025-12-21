import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import Translate from '@docusaurus/Translate';
import useBaseUrl from '@docusaurus/useBaseUrl';

type BackgroundFormData = {
  programmingExperience: string;
  hardwareExperience: string;
  roboticsBackground: string;
  interests: string[];
};

const SignupFormContent = () => {
  const [email, setEmail] = React.useState('');
  const [password, setPassword] = React.useState('');
  const [confirmPassword, setConfirmPassword] = React.useState('');
  const [programmingExperience, setProgrammingExperience] = React.useState('beginner');
  const [hardwareExperience, setHardwareExperience] = React.useState('none');
  const [roboticsBackground, setRoboticsBackground] = React.useState('none');
  const [interests, setInterests] = React.useState<string[]>([]);
  const [error, setError] = React.useState('');
  const [loading, setLoading] = React.useState(false);
  const history = require('@docusaurus/router').useHistory();
  const profileUrl = useBaseUrl('/user-profile');

  // Define navigate function using history
  const navigate = (path) => {
    history.push(path);
  };

  const handleInterestChange = (interest: string) => {
    if (interests.includes(interest)) {
      setInterests(interests.filter(i => i !== interest));
    } else {
      setInterests([...interests, interest]);
    }
  };

  const validateForm = () => {
    if (password !== confirmPassword) {
      setError('Passwords do not match');
      return false;
    }
    if (password.length < 8) {
      setError('Password must be at least 8 characters');
      return false;
    }
    setError('');
    return true;
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!validateForm()) return;

    setLoading(true);
    setError('');

    try {
      // Use the mock signup function that properly stores user data
      const mockSignUp = require('../auth/client').mockSignUp;
      const result = await mockSignUp(email, password, {
        programmingExperience,
        hardwareExperience,
        roboticsBackground,
        interests: interests.join(',')
      });

      if (result.error) {
        setError(result.error.message);
      } else {
        // Redirect to user profile page after successful signup
        navigate(profileUrl);
      }
    } catch (err: any) {
      setError(err.message || 'An error occurred during signup');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="signup-form-container">
      <h2><Translate id="signup.formTitle">Create Your Account</Translate></h2>
      <p><Translate id="signup.subtitle">Help us personalize your learning experience by sharing your background</Translate></p>

      {error && <div className="alert alert--danger">{error}</div>}

      <form onSubmit={handleSubmit}>
        <div className="form-group">
          <label htmlFor="email"><Translate id="signup.email">Email</Translate>:</label>
          <input
            type="email"
            id="email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            required
          />
        </div>

        <div className="form-group">
          <label htmlFor="password"><Translate id="signup.password">Password</Translate>:</label>
          <input
            type="password"
            id="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            required
            minLength={8}
          />
        </div>

        <div className="form-group">
          <label htmlFor="confirmPassword"><Translate id="signup.confirmPassword">Confirm Password</Translate>:</label>
          <input
            type="password"
            id="confirmPassword"
            value={confirmPassword}
            onChange={(e) => setConfirmPassword(e.target.value)}
            required
          />
        </div>

        <hr />

        <h3><Translate id="signup.backgroundTitle">Your Background & Interests</Translate></h3>

        <div className="form-group">
          <label><Translate id="signup.programmingExp">Programming Experience</Translate>:</label>
          <div className="radio-group">
            {['beginner', 'intermediate', 'advanced'].map((level) => (
              <label key={level} className="radio-option">
                <input
                  type="radio"
                  name="programmingExperience"
                  value={level}
                  checked={programmingExperience === level}
                  onChange={() => setProgrammingExperience(level)}
                />
                {level === 'beginner' ? <Translate id="signup.level.beginner">Beginner</Translate> :
                 level === 'intermediate' ? <Translate id="signup.level.intermediate">Intermediate</Translate> :
                 <Translate id="signup.level.advanced">Advanced</Translate>}
              </label>
            ))}
          </div>
        </div>

        <div className="form-group">
          <label><Translate id="signup.hardwareExp">Hardware Experience</Translate>:</label>
          <div className="radio-group">
            {['none', 'basic', 'intermediate', 'advanced'].map((level) => (
              <label key={level} className="radio-option">
                <input
                  type="radio"
                  name="hardwareExperience"
                  value={level}
                  checked={hardwareExperience === level}
                  onChange={() => setHardwareExperience(level)}
                />
                {level === 'none' ? <Translate id="signup.level.none">None</Translate> :
                 level === 'basic' ? <Translate id="signup.level.basic">Basic</Translate> :
                 level === 'intermediate' ? <Translate id="signup.level.intermediate">Intermediate</Translate> :
                 <Translate id="signup.level.advanced">Advanced</Translate>}
              </label>
            ))}
          </div>
        </div>

        <div className="form-group">
          <label><Translate id="signup.roboticsBackground">Robotics Background</Translate>:</label>
          <div className="radio-group">
            {['none', 'academic', 'industry'].map((level) => (
              <label key={level} className="radio-option">
                <input
                  type="radio"
                  name="roboticsBackground"
                  value={level}
                  checked={roboticsBackground === level}
                  onChange={() => setRoboticsBackground(level)}
                />
                {level === 'none' ? <Translate id="signup.robotics.none">None</Translate> :
                 level === 'academic' ? <Translate id="signup.robotics.academic">Academic</Translate> :
                 <Translate id="signup.robotics.industry">Industry</Translate>}
              </label>
            ))}
          </div>
        </div>

        <div className="form-group">
          <label><Translate id="signup.interests">Interests</Translate>:</label>
          <div className="checkbox-group">
            {['ROS 2', 'Simulation', 'NVIDIA Isaac', 'RL', 'Humanoids'].map((interest) => (
              <label key={interest} className="checkbox-option">
                <input
                  type="checkbox"
                  checked={interests.includes(interest)}
                  onChange={() => handleInterestChange(interest)}
                />
                {interest}
              </label>
            ))}
          </div>
        </div>

        <button type="submit" disabled={loading} className="button button--primary">
          {loading ? <Translate id="signup.creating">Creating Account...</Translate> : <Translate id="signup.signUpBtn">Sign Up</Translate>}
        </button>
      </form>
    </div>
  );
};

const SignupForm = () => {
  return (
    <BrowserOnly>
      {() => <SignupFormContent />}
    </BrowserOnly>
  );
};

export default SignupForm;

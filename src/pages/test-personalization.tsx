import React from 'react';
import { useAuth } from '../contexts/AuthContext';
import PersonalizedContent from '../components/PersonalizedContent';
import DifficultyIndicator from '../components/DifficultyIndicator';
import OnboardingContent from '../components/OnboardingContent';
import BrowserOnly from '@docusaurus/BrowserOnly';

const TestPersonalizationPageContent: React.FC = () => {
  const { user, loading } = useAuth();

  if (loading) {
    return <div>Loading...</div>;
  }

  return (
    <div className="test-personalization-page">
      <h1>Personalization Test Page</h1>

      <h2>User Information</h2>
      {user ? (
        <div>
          <p><strong>Email:</strong> {user.email}</p>
          <p><strong>Programming Experience:</strong> {user.background.programmingExperience}</p>
          <p><strong>Hardware Experience:</strong> {user.background.hardwareExperience}</p>
          <p><strong>Robotics Background:</strong> {user.background.roboticsBackground}</p>
          <p><strong>Interests:</strong> {user.background.interests?.join(', ') || 'No interests selected'}</p>
        </div>
      ) : (
        <p>User is not authenticated. Some content will be visible to all, some only to logged-in users.</p>
      )}

      <h2>Onboarding Recommendations</h2>
      <OnboardingContent />

      <h2>Personalized Content Examples</h2>

      <div className="content-section">
        <h3>Content for Beginners</h3>
        <PersonalizedContent forProgrammingLevel="beginner">
          <div className="personalized-content">
            <h4>Beginner-Friendly Content</h4>
            <p>This content is specifically designed for users with beginner programming experience.</p>
            <DifficultyIndicator level="beginner" />
          </div>
        </PersonalizedContent>
        <p>If you're not a beginner, you won't see the above content.</p>
      </div>

      <div className="content-section">
        <h3>Content for Hardware Experts</h3>
        <PersonalizedContent forHardwareLevel="advanced">
          <div className="personalized-content">
            <h4>Advanced Hardware Content</h4>
            <p>This content is designed for users with advanced hardware experience.</p>
            <DifficultyIndicator level="advanced" />
          </div>
        </PersonalizedContent>
        <p>If you don't have advanced hardware experience, you won't see the above content.</p>
      </div>

      <div className="content-section">
        <h3>Content for ROS 2 Interests</h3>
        <PersonalizedContent forInterest="ROS 2">
          <div className="personalized-content">
            <h4>ROS 2 Specific Content</h4>
            <p>This content is tailored for users interested in ROS 2.</p>
            <DifficultyIndicator level="intermediate" />
          </div>
        </PersonalizedContent>
        <p>If you didn't select ROS 2 as an interest, you won't see the above content.</p>
      </div>

      <div className="content-section">
        <h3>Content for Everyone</h3>
        <div className="personalized-content">
          <h4>General Content</h4>
          <p>This content is visible to all users regardless of their background.</p>
        </div>
      </div>
    </div>
  );
};

const TestPersonalizationPage: React.FC = () => {
  return (
    <BrowserOnly>
      {() => <TestPersonalizationPageContent />}
    </BrowserOnly>
  );
};

export default TestPersonalizationPage;

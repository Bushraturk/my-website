import React, { useState, useEffect } from 'react';
import { useAuth } from '../contexts/AuthContext';
import BonusPointsDisplay from './BonusPointsDisplay';

interface ChapterPersonalizationButtonProps {
  chapterId: string;
  children: React.ReactNode;
  fallbackContent?: React.ReactNode; // Content to show if not personalized
}

const ChapterPersonalizationButton: React.FC<ChapterPersonalizationButtonProps> = ({
  chapterId,
  children,
  fallbackContent
}) => {
  const { user, loading } = useAuth();
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [personalizedContent, setPersonalizedContent] = useState<React.ReactNode>(null);
  const [showBonus, setShowBonus] = useState(false);

  // Check if personalization has been applied for this chapter
  useEffect(() => {
    if (user && !loading) {
      const personalizationStatus = localStorage.getItem(`chapter-${chapterId}-personalized`);
      if (personalizationStatus === 'true') {
        setIsPersonalized(true);
      }
    }
  }, [user, loading, chapterId]);

  const handlePersonalize = () => {
    if (!user) {
      alert('Please sign in to access personalized content');
      return;
    }

    // Mark as personalized in localStorage
    localStorage.setItem(`chapter-${chapterId}-personalized`, 'true');
    setIsPersonalized(true);

    // Show bonus points display
    setShowBonus(true);

    // In a real implementation, we would transform the content based on user's profile
    // For now, we'll just display the provided personalized content
    setPersonalizedContent(children);
  };

  if (loading) {
    return <div>Loading personalization options...</div>;
  }

  if (!user) {
    // If user is not authenticated, show sign-in message
    return (
      <div className="chapter-personalization-prompt">
        <div className="alert alert--info">
          Sign in to get personalized content based on your background and interests.
        </div>
        {fallbackContent}
      </div>
    );
  }

  if (isPersonalized) {
    // If already personalized, show personalized content with bonus points indicator
    return (
      <div className="personalized-content">
        {showBonus && <BonusPointsDisplay points={50} message="Bonus points earned for personalizing content!" />}
        {personalizedContent || children}
      </div>
    );
  }

  // If not personalized yet, show the button
  return (
    <div className="chapter-personalization-container">
      <div className="personalization-prompt">
        <div className="personalization-header">
          <h3>Personalize This Chapter</h3>
          <p>Get content tailored to your background and interests:</p>
          <ul>
            <li>Programming experience: {user.background.programmingExperience}</li>
            <li>Hardware experience: {user.background.hardwareExperience}</li>
            <li>Robotics background: {user.background.roboticsBackground}</li>
            <li>Interests: {user.background.interests.join(', ')}</li>
          </ul>
        </div>
        <button
          className="button button--primary personalization-button"
          onClick={handlePersonalize}
        >
          Personalize Content (Get Bonus Points!)
        </button>
        <div className="personalization-info">
          <p>This will customize the content below based on your profile. Up to 50 bonus points available!</p>
        </div>
      </div>
      <div className="non-personalized-content">
        {fallbackContent}
      </div>
    </div>
  );
};

export default ChapterPersonalizationButton;
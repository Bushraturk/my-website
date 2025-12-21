import React, { useState, useEffect } from 'react';

interface PersonalizedChapterContentProps {
  chapterTitle: string;
  baseContent: string;
}

const PersonalizedChapterContent: React.FC<PersonalizedChapterContentProps> = ({
  chapterTitle,
  baseContent
}) => {
  const [userData, setUserData] = useState<any>(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    // Check authentication state from localStorage
    const mockUserData = localStorage.getItem('mockUser');
    if (mockUserData) {
      try {
        const parsedData = JSON.parse(mockUserData);
        setUserData(parsedData);
      } catch (e) {
        console.error('Error parsing user data:', e);
      }
    }
    setLoading(false);
  }, []);

  if (loading) {
    return <div>Loading personalized content...</div>;
  }

  if (!userData) {
    return <div>{baseContent}</div>; // Show base content if not logged in
  }

  // Personalize content based on user's profile
  let personalizedContent = baseContent;

  // Example personalization logic:
  // Adjust content based on user's programming experience
  if (userData.background?.programmingExperience === 'beginner') {
    personalizedContent = `BEGINNER LEVEL: ${baseContent}`;
  } else if (userData.background?.programmingExperience === 'advanced') {
    personalizedContent = `ADVANCED LEVEL: ${baseContent} - This builds on your existing knowledge.`;
  }

  // Adjust content based on user's robotics background
  if (userData.background?.roboticsBackground === 'academic') {
    personalizedContent += ` [Academic focus: More theoretical concepts and research.]`;
  } else if (userData.background?.roboticsBackground === 'industry') {
    personalizedContent += ` [Industry focus: Practical applications and real-world implementations.]`;
  }

  // Adjust content based on user's interests
  if (userData.background?.interests && Array.isArray(userData.background.interests) &&
      userData.background.interests.includes('ROS 2')) {
    personalizedContent += ` [ROS 2 Reference: For a deeper look at this concept, check out ROS 2 specific examples in our ROS 2 module.]`;
  }

  if (userData.background?.interests && Array.isArray(userData.background.interests) &&
      userData.background.interests.includes('NVIDIA Isaac')) {
    personalizedContent += ` [NVIDIA Isaac Reference: You might find this concept implemented in NVIDIA Isaac examples.]`;
  }

  return (
    <div className="personalized-chapter-content">
      <div className="personalization-info-box">
        <p><strong>Personalized for your profile:</strong></p>
        <ul>
          <li>Experience level: {userData.background?.programmingExperience || 'Not specified'}</li>
          <li>Hardware background: {userData.background?.hardwareExperience || 'Not specified'}</li>
          <li>Robotics background: {userData.background?.roboticsBackground || 'Not specified'}</li>
          <li>Interests: {userData.background?.interests && Array.isArray(userData.background.interests) ?
            userData.background.interests.join(', ') : 'Not specified'}</li>
        </ul>
      </div>
      <div className="content-body">
        {personalizedContent}
      </div>
    </div>
  );
};

export default PersonalizedChapterContent;